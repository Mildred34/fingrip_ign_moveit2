#!/usr/bin/env -S ros2 launch
import os
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.events.matchers import matches_action
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Parameters that doesn't depend of config files
    namespace = LaunchConfiguration("namespace")
    nodename = LaunchConfiguration("nodename")
    port = LaunchConfiguration("port")
    executable = LaunchConfiguration("executable")
    no_collision = LaunchConfiguration("no_collision")

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

    # List of event handlers
    event_handlers = []

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
                    ("nodename", nodename),
                    ("port", port),
                    ("executable", executable),
                ],
                condition=UnlessCondition(no_collision),
            )
        )
        launch_descriptions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare(package_name),
                            "launch",
                            "view_coppelia_nocollision.launch.py",
                        ]
                    )
                ),
                launch_arguments=[
                    ("namespace", namespace),
                    ("nodename", nodename),
                    ("port", port),
                    ("executable", executable),
                ],
                condition=IfCondition(no_collision),
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
                    ("nodename", nodename),
                    ("port", port),
                    ("no_collision", no_collision),
                    ("executable", executable),
                ],
            )
        )

    event_handlers.append(
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    EmitEvent(
                        event=ShutdownProcess(
                            process_matcher=matches_action(
                                launch_descriptions[-1]
                            )
                        )
                    )
                ],
            )
        )
    )

    return LaunchDescription(
        declared_arguments + launch_descriptions + event_handlers
    )


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
            default_value="sim1",
            description="Namespace use for simulation topics avoiding \
                collision",
        ),
        DeclareLaunchArgument(
            "nodename",
            default_value="sim1",
            description="Node name within sim",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="23000",
            description="Simulator remote server will be launched \
                on this port",
        ),
        DeclareLaunchArgument(
            "no_collision",
            default_value="False",
            description="If True Simulator will be launched without \
                collision between gripper and object",
        ),
        DeclareLaunchArgument(
            "executable",
            default_value="coppeliaSim.sh",
            description="Has to launch simulator from different directory \
                for multiple simulators launching",
        ),
    ]
