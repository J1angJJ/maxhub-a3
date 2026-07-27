from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare("carm_a3_description"),
        "urdf",
        "carm_a3.urdf",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("carm_a3_description"),
        "rviz",
        "default.rviz",
    ])

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
        DeclareLaunchArgument("rviz_config", default_value=rviz_config),
        DeclareLaunchArgument("publish_frequency", default_value="50.0"),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            arguments=[LaunchConfiguration("urdf")],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ])
