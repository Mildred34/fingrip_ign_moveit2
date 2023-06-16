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

    # List of included launch descriptions
    # Launch Coppelia
    model_path = PathJoinSubstitution(
                    [
                        FindPackageShare([package_name]),
                        "model",
                        scene,
                    ]
                )
    
    coppelia = ExecuteProcess(
            cmd=[[
                FindExecutable(name='coppeliaSim.sh'),
                ' '
                ,
                model_path
            ]],
            shell=True
        )
    
    launch_descriptions = [
            coppelia,
    ]

    return LaunchDescription(declared_arguments + launch_descriptions)
    # return LaunchDescription()


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
            default_value="robotiq-assembly-V4.ttt",
            description="Name or filepath of model to load.",
        ),
        # Robot selection
        DeclareLaunchArgument(
            "robot_type",
            default_value="fingrip",
            description="Name of the robot type to use.",
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