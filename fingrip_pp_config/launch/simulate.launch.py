#!/usr/bin/env -S ros2 launch
import os
from typing import List
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Parameters that doesn't depend of config files
    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")

    # Get Config
    config_path = os.path.join(
        get_package_share_directory("fingrip_description"),
        "config",
        "config_global.yaml",
    )
    config = None

    with open(config_path, "r") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    # Get substitution for all arguments
    package_name = config["description_package"]
    headless_mode = config["headless_mode"]

    # List of included launch descriptions
    launch_descriptions = []

    # List of config files
    # object_config = PathJoinSubstitution(
    #     [
    #         FindPackageShare(package_name),
    #         "config",
    #         "object_config.yaml",
    #     ]
    # )

    # object_model_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare(package_name),
    #         "resource",
    #     ]
    # )

    # Launch Coppelia
    if not headless_mode:
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
                    ("namespace", namespace),
                    ("port", port),
                ],
            )
        )
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
                    ("namespace", namespace),
                    ("port", port),
                ],
            )
        )

    return LaunchDescription(declared_arguments + launch_descriptions)


# Centralize all arguments in a config file. Doesn't use anyme
# the command pass arguments
def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for
    this launch script.
    """
    return [
        # Simulation
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace use for simulation topics avoiding \
                collision",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="23000",
            description="Simulator remote server will be launched \
                on this port",
        ),
    ]
