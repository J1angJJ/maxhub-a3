import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("carm_a3_description")
    gazebo_share = get_package_share_directory("carm_gazebo")
    urdf_file = os.path.join(description_share, "urdf", "carm_a3.urdf")
    world_file = os.path.join(gazebo_share, "worlds", "empty.world")
    gazebo_model_path = os.path.dirname(description_share)

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            LaunchConfiguration("urdf"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=urdf_file),
        DeclareLaunchArgument("world", default_value=world_file),
        DeclareLaunchArgument("entity", default_value="carm_a3"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.0"),
        ExecuteProcess(
            cmd=[
                "gzserver",
                "--verbose",
                "--pause",
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
        ),
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
        Node(
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
        ),
    ])
