import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = True

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("nsc_description"), "launch", "description.launch.py"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("nsc_description"), "worlds", "world.world"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world", default_value=world_path, description="Gazebo world"
            ),
            DeclareLaunchArgument(
                name="rtabmap", default_value="true", description="Run rtabmap"
            ),
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "--verbose",
                    "-s",
                    "libgazebo_ros_factory.so",
                    "-s",
                    "libgazebo_ros_init.so",
                    LaunchConfiguration("world"),
                ],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="urdf_spawner",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-entity",
                    "nsc_bot",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch_path),
                launch_arguments={
                    "use_sim_time": str(use_sim_time),
                    "publish_joints": "false",
                    "publish_controller": "true",
                    "rviz": "false",
                }.items(),
            ),
        ]
    )
