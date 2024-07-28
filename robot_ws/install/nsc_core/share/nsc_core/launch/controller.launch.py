import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    arm_control = Node(
        package="nsc_core",
        executable="joint_control.py",
    )
    camera_control = Node(
        package="nsc_core",
        executable="camera_control.py",
    )

    ld.add_action(arm_control)
    ld.add_action(camera_control)

    return ld
