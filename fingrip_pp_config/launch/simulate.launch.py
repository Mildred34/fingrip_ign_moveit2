#!/usr/bin/env -S ros2 launch
"""Visualisation of SDF model for fingrip in Ignition Gazebo. Note that the generated model://fingrip/model.sdf descriptor is used."""

from os import path, getcwd
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    scene = LaunchConfiguration("scene")
    log_level = LaunchConfiguration("log_level")
    robot_type = LaunchConfiguration("robot_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_name = LaunchConfiguration("description_package")
    object_type = LaunchConfiguration("object_type")

    # List of included launch descriptions
    launch_descriptions = []

    # List of config files
    object_config = PathJoinSubstitution(
                [
                    FindPackageShare("fingrip_description"),
                    "config",
                    "object_config.yaml",
                ]
            )
    
    object_model_path = PathJoinSubstitution(
                [
                    FindPackageShare("fingrip_description"),
                    "resource",
                ]
            )

    # Launch Coppelia
    launch_descriptions.append(
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fingrip_description"),
                    "launch",
                    "view_coppelia.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("scene", scene),
            ("description_package", package_name),
        ],
    ))

    # Launch simulation
    nodes = [
        Node(
            package="fingrip_pp_config",
            executable="simulate_node",
            output="both",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {"object_type":object_type},
                {"object_config":object_config},
                {"object_model_path":object_model_path},
                # "use_sim_time":use_sim_time},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="fingrip_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="robotiq-assembly-V7.ttt",
            description="Name or filepath of model to load.",
        ),
        # Robot selection
        DeclareLaunchArgument(
            "robot_type",
            default_value="fingrip",
            description="Name of the robot type to use.",
        ),
        # Object selection
        DeclareLaunchArgument(
            "object_type",
            default_value="renfort1",
            description="Object that we will test it out",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
