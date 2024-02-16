#!/usr/bin/env -S ros2 launch
import os
import signal
import subprocess
from typing import Iterable, List, Text

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.events.matchers import matches_action
from launch.events.process import ShutdownProcess
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.substitutions import FindPackageShare


class TextFormatting(Substitution):
    """Substitution that wraps a single string text.

    Example:
    command = TextFormatting(
         "{} -GzmqRemoteApi.rpcPort={} -GROS2Interface.nodeName={} -g{} {}",
         [FindExecutable(name=simulator_exe), port, nodename, namespace, model_path],
     )
    """

    def __init__(
        self, text: Text, substitutions: Iterable[SomeSubstitutionsType]
    ) -> None:
        """Create a TextSubstitution."""
        super().__init__()

        if not isinstance(text, Text):
            raise TypeError(
                "TextSubstitution expected Text object got '{}' instead.".format(
                    type(text)
                )
            )

        self.__text = text
        self.__substitutions = normalize_to_list_of_substitutions(
            substitutions
        )

    @property
    def substitutions(self) -> Iterable[Substitution]:
        """Getter for variable_name."""
        return self.__substitutions

    @property
    def text(self) -> Text:
        """Getter for text."""
        return self.__text

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return "{}%({})".format(
            self.text, ", ".join([s.describe() for s in self.substitutions])
        )

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        performed_substitutions = [
            sub.perform(context) for sub in self.__substitutions
        ]
        return self.text.format(*performed_substitutions)


def on_terminate(_launch_context: LaunchContext, cmd: Substitution):
    """Function use to kill all coppelia Simulators launched
    Indeed coppelia is launched through a shell script.
    ROS will kill the shell pid but not the simulator one.

    Args:
        _launch_context (_type_): _description_
    """
    p = subprocess.Popen(["ps", "-A"], stdout=subprocess.PIPE)
    out, err = p.communicate()
    cmd_text = cmd.perform(_launch_context)
    print(f"Going to kill all instances of:{cmd_text}")

    for line in out.splitlines():
        if cmd_text in str(line):
            pid = int(line.split(None, 1)[0])
            os.kill(pid, signal.SIGKILL)


def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Parameters that doesn't depend of config files
    namespace = LaunchConfiguration("namespace")
    nodename = LaunchConfiguration("nodename")
    port = LaunchConfiguration("port")
    simulator_exe = LaunchConfiguration("executable")

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
    scene = config["scene"]
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
    """ Previous way to execute the simulator with shell"""
    # coppelia = ExecuteProcess(
    #     cmd=[
    #         [
    #             FindExecutable(name=simulator_exe),
    #             " ",
    #             "-GzmqRemoteApi.rpcPort=",
    #             port,
    #             " ",
    #             "-GROS2Interface.nodeName=",
    #             nodename,
    #             " ",
    #             "-g",
    #             namespace,
    #             " ",
    #             model_path,
    #         ]
    #     ],
    #     shell=True,
    # )

    cmd_port = TextFormatting("-GzmqRemoteApi.rpcPort={}", port)
    cmd_nodename = TextFormatting("-GROS2Interface.nodeName={}", nodename)
    cmd_namespace = TextFormatting("-g{}", namespace)

    coppelia = ExecuteProcess(
        cmd=[
            FindExecutable(name=simulator_exe),
            cmd_port,
            cmd_nodename,
            cmd_namespace,
            model_path,
        ],
        shell=False,
        # On exit is here to kill all child process create by the simulation
        on_exit=OpaqueFunction(function=on_terminate, args=[simulator_exe]),
    )

    launch_descriptions = [
        coppelia,
    ]

    event_handlers = []
    # Destroy simulator
    event_handlers.append(
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    EmitEvent(
                        event=ShutdownProcess(
                            process_matcher=matches_action(coppelia)
                        )
                    ),
                ],
            )
        )
    )

    return LaunchDescription(
        declared_arguments + launch_descriptions + event_handlers
    )


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
            description="Simulator remote server will be launched on \
                this port",
        ),
        DeclareLaunchArgument(
            "executable",
            default_value="coppeliaSim",
            description="Has to launch simulator from different directory \
                for multiple simulators launching",
        ),
    ]
