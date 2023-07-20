#!/usr/bin/env -S ros2 launch
import os
from os import path, getcwd
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from launch.conditions import IfCondition,UnlessCondition


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
    log_level = config["simulator_log_level"]
    package_name = config["description_package"]
    object_type = config["object_type"]
    headless_mode = config["headless_mode"]
    
    # Parameters that doesn't depend of config files
    namespace = LaunchConfiguration("namespace")

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
    
    # Parameters that doesn't depend of config files
    namespace = LaunchConfiguration("namespace")
    
    # Launch Coppelia
    if( not headless_mode):
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
                ("namespace",namespace),
            ],
        ))
    else:
        launch_descriptions.append(
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare(package_name),
                        "launch",
                        "view_coppelia_headless.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("namespace",namespace),
            ],
        ))

    # Launch simulation
    nodes = [
        Node(
            package="fingrip_pp_config",
            executable="simulate_node",
            namespace=namespace,
            output="both",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {"object_type":object_type},
                {"object_config":object_config},
                {"object_model_path":object_model_path},
            ],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)

# Centralize all arguments in a config file. Doesn't use anyme the command pass arguments
def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Simulation
        DeclareLaunchArgument(
            "namespace",
            default_value="sim_2",
            description="Namespace use for simulation topics avoiding collision",
        ),
    ]
