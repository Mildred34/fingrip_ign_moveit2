# fingrip_description

URDF and SDF description of Fingrip.

Here add picture of the fingrip model :
<p align="center" float="middle">
  <img width="50.0%" src="fingrip/thumbnails/2.png" alt="Visualisation of fingrip visual and collision geometry"/>
</p>

## Instructions

### URDF

For URDF, [fingrip.urdf.xacro](./urdf/fingrip.urdf.xacro) is the primary descriptor that includes all other xacros and creates a model based on the passed arguments.
To generate URDF out of xacro, you can use the included [xacro2urdf.bash](./scripts/xacro2urdf.bash) script and modify its arguments as needed.
Once executed, [fingrip.urdf](./urdf/fingrip.urdf) will automatically be replaced.
Alternatively, `xacro fingrip.urdf.xacro name:="fingrip" <arg_i>:=<val_i> ...` can be executed directly, e.g. this is preferred within any launch script.

In order to visualise URDF with RViz2, included [view.launch.py](./launch/view.launch.py) script can be used.

```bash
ros2 launch fingrip_description view.launch.py
```

### SDF

For SDF, please use the included [xacro2sdf.bash](./scripts/xacro2sdf.bash) script with the desired arguments.
This script makes sure that a correct relative path is used to locate all assets.

To visualise SDF with Gazebo, included [view_ign.launch.py](./launch/view_ign.launch.py) script can be used.

```bash
ros2 launch fingrip_description view_ign.launch.py
```

#### Fuel

If you do not require URDF and other resources from this repository, the default model (without `ros2_control`)
can also be included directly from [Fuel](https://app.gazebosim.org/AndrejOrsula/fuel/models/fingrip)
if you do not require the URDF description.

```xml
<include>
    <uri>https://fuel.gazebosim.org/1.0/AndrejOrsula/models/panda</uri>
</include>
```

## Disclaimer

Several of the included xacros and meshes originated in [frankaemika/franka_ros](https://github.com/frankaemika/franka_ros/tree/develop/franka_description).
These files were modified to fit the purpose of this repository, e.g. xacros were refactored,
inertial properties were estimated, support for `ros2_control` was added, mesh geometry was remodelled to improve performance, ...

## Directory Structure

The following directory structure is utilised for this package because it provides
compatibility with Gazebo, including [Fuel](https://app.gazebosim.org).

```bash
.
├── config/initial_joint_positions.yaml # List of initial joint positions for fake and simulated ROS 2 control
├── launch/                             # [dir] ROS 2 launch scripts
    ├── view.launch.py                  # Launch script for visualising URDF with RViz2
    └── view_ign.launch.py              # Launch script for visualising SDF with Gazebo
├── fingrip/                              # [dir] Model directory compatible with Fuel
    ├── meshes/                         # [dir] Meshes for both URDF and SDF
        ├── **/collision/*.stl          # STL meshes for collision geometry
        └── **/visual/*.dae             # COLLADA meshes for visuals
    ├── thumbnails/                     # [dir] Thumbnails for Fuel
    ├── model.config                    # Model meta data
    └── model.sdf                       # SDF (generated from URDF)
├── rviz/view.rviz                      # RViz2 config for visualising URDF
├── scripts/                            # [dir] Additional useful scripts
├── urdf/                               # [dir] URDF description (xacros)
    ├── fingrip_arm.xacro                 # Xacro for Franka Emika fingrip arm
    ├── fingrip_gripper.xacro             # Xacro for Franka Emika fingrip gripper
    ├── fingrip_inertial.xacro            # Macro for inertial properties of Franka Emika fingrip
    ├── fingrip_utils.Macro               # Macros for general boilerplate
    ├── fingrip.gazebo                    # Macros that add Gazebo plugins for Franka Emika fingrip
    ├── fingrip.ros2_control              # Macros that add ros2 control for Franka Emika fingrip
    ├── fingrip.urdf                      # URDF (generated from fingrip.urdf.xacro)
    └── fingrip.urdf.xacro                # High-level xacro for Franka Emika fingrip
├── CMakeLists.txt                      # Colcon-enabled CMake recipe
└── package.xml                         # ROS 2 package metadata
```
