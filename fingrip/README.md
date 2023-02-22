# fingrip

Metapackage for fingrip.

## Functionality

During the build stage, this package converts xacros of [fingrip_description](../fingrip_description) and [fingrip_moveit_config](../fingrip_moveit_config) into auto-generated URDF, SDF and SRDF descriptions for convenience.

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── CMakeLists.txt # Colcon-enabled CMake recipe
└── package.xml    # ROS 2 package metadata
```
