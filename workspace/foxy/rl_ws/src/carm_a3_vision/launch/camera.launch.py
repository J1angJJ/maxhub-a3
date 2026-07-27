import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("carm_a3_vision")
    default_config = os.path.join(share_dir, "config", "camera.yaml")
    default_camera_info_url = "file://" + os.path.join(share_dir, "config", "camera_info.yaml")

    config = LaunchConfiguration("config")
    camera_info_url = LaunchConfiguration("camera_info_url")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("camera_info_url", default_value=default_camera_info_url),
        Node(
            package="carm_a3_vision",
            executable="v4l2_camera_node",
            name="carm_a3_usb_camera",
            output="screen",
            parameters=[
                config,
                {"camera_info_url": camera_info_url},
            ],
        ),
    ])
