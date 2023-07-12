#!/usr/bin/env -S ros2 launch
from os import path, getcwd
import os
from typing import List
import yaml

from ament_index_python.packages import get_package_share_directory
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

    # Get Config
    config_path = os.path.join(
        get_package_share_directory('fingrip_description'),
        'config',
        'config_global.yaml'
        )
    config = None
    
    with open(config_path,'r') as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                
                
    # Get substitution for all arguments
    package_name = config["description_package"]

    # List of included launch descriptions
    launch_descriptions = []

    # List of config files
    object_config = PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "config",
                    "object_config.yaml",
                ]
            )
    
    object_model_path = PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "resource",
                ]
            )

    # Launch Coppelia
    launch_descriptions.append(
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "launch",
                    "view_coppelia.launch.py",
                ]
            )
        ),
        launch_arguments=[
        ],
    ))

    return LaunchDescription(declared_arguments + launch_descriptions)

# Centralize all arguments in a config file. Doesn't use anyme the command pass arguments
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
