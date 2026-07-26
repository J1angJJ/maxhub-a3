from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare("carm_rl_bringup"),
        "config",
        "carm_api.yaml",
    ])

    return LaunchDescription([
        Node(
            package="carm_api",
            executable="carm_ros_node",
            name="carm_api",
            output="screen",
            parameters=[params],
        ),
    ])
