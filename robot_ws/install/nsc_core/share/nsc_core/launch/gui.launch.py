import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    gui = Node(
        package="nsc_core",
        executable="gui.py",
    )
    video_capture = Node(
        package="nsc_core",
        executable="video_capture.py",
    )
    ld.add_action(gui)
    ld.add_action(video_capture)

    return ld
