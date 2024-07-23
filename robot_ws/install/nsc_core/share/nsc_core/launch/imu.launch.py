import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    imu_filter_node1 = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu1",
        parameters=[
            {
                "use_mag": False,
                # "gain": 0.0,
                "fixed_frame": "imu_link",
            }
        ],
        remappings=[
            ("/imu/data_raw", "/imu1/data"),
            ("/imu/data", "/imu1/filtered"),
        ],
    )

    imu_filter_node2 = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu2",
        parameters=[
            {
                "use_mag": False,
                # "gain": 0.0,
                "fixed_frame": "imu_link",
            }
        ],
        remappings=[
            ("/imu/data_raw", "/imu2/data"),
            ("/imu/data", "/imu2/filtered"),
        ],
    )

    ld.add_action(imu_filter_node1)
    ld.add_action(imu_filter_node2)

    return ld
