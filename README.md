# fingrip_simulation

Software packages for fingrip that enable manipulation with OMPL inside ~~CoppeliaSim~~.

<p align="left" float="middle">
  <img width="50.0%" src="https://user-images.githubusercontent.com/22929099/147374612-3d0209d3-574e-4a4f-8077-edbbcf8fc47d.gif" alt="Animation of ex_follow_target"/>
</p>

## Overview

This branch targets ROS 2 `humble` and CoppeliaSim V4.6.

Below is an overview of the included packages, with a short description of their purpose. For more information, please see README.md of each individual package.

- [**fingrip_description**](./fingrip_description) – coppeliasim scene that contains our robot
- [**fingrip_pp_config**](./fingrip_pp_config) – Interact with the coppeliasim simulator. I will add some configuration for the path planning afterwards

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [humble](https://docs.ros.org/en/humblr/Installation.html)
- Coppeliasim [4.5](https://www.coppeliarobotics.com/downloads)
Add this to your bashrc file:
```
export PATH="$HOME/CoppeliaSim_Edu_V4_6_0_rev6_Ubuntu22_04:$PATH"
export COPPELIASIM_ROOT_DIR="$HOME/CoppeliaSim_Edu_V4_6_0_rev6_Ubuntu22_04"
alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"
```

First line is to allow your computer to search with the folder for binary file.
The last one, is to change the name of shell script to launch coppelia by just *coppelia*.

So youcan now launch coppeliasim with the following command:
```bash
coppelia
```

### Building

Clone this repository, import dependencies, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone

# Import dependencies


# Install dependencies
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .

# Build
colcon build
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables:

- Execution of binaries, scripts and examples via `ros2 run fingrip_* <executable>`
- Launching of setup scripts via `ros2 launch fingrip_* <launch_script>`
- Discoverability of shared resources
