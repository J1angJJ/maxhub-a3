import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory("carm_gazebo")
    description_share = get_package_share_directory("carm_a3_description")
    urdf_file = os.path.join(gazebo_share, "urdf", "carm_a3_gazebo.urdf.xacro")
    world_file = os.path.join(gazebo_share, "worlds", "empty.world")
    gazebo_model_path = os.pathsep.join([
        os.path.dirname(description_share),
        os.environ.get("GAZEBO_MODEL_PATH", ""),
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("urdf"),
        ]),
        value_type=str,
    )

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            LaunchConfiguration("world"),
        ],
        additional_env={
            "GAZEBO_MODEL_DATABASE_URI": "",
            "GAZEBO_MODEL_PATH": gazebo_model_path,
        },
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_carm_a3",
        output="screen",
        arguments=[
            "-entity",
            LaunchConfiguration("entity"),
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=urdf_file),
        DeclareLaunchArgument("world", default_value=world_file),
        DeclareLaunchArgument("entity", default_value="carm_a3"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        gzserver,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0,
            }],
        ),
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    arm_controller_spawner,
                ],
            )
        ),
    ])
