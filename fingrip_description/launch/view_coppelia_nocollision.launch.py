#!/usr/bin/env -S ros2 launch
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

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
    scene = config["scene_nocollision"]
    package_name = config["description_package"]

    # List of included launch descriptions
    # Launch Coppelia
    model_path = PathJoinSubstitution(
        [
            FindPackageShare([package_name]),
            "scene",
            scene,
        ]
    )

    # Parameters that doesn't depend of config files
    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")

    coppelia_launch = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name="coppeliaSim.sh"),
                " ",
                "-GzmqRemoteApi.rpcPort=",
                port,
                " ",
                "-GROS2Interface.nodeName=",
                namespace,
                " ",
                "-g",
                namespace,
                " ",
                model_path,
            ]
        ],
        shell=True,
    )

    launch_descriptions = [coppelia_launch]

    return LaunchDescription(declared_arguments + launch_descriptions)


# Centralize all arguments in a config file. Doesn't use anyme the command
# pass arguments
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
            description="Simulator remote server will be launched on \
                this port",
        ),
    ]
