cmake_minimum_required(VERSION 3.5)
project(fingrip_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Setup environment hooks that automatically prepend IGN_GAZEBO_RESOURCE_PATH (now GZ_SIM_RESOURCE_PATH) and SDF_PATH when sourcing local_setup.<ext>
ament_environment_hooks(
  "cmake/environment_hooks/ign_gazebo_resource_path.bash"
  "cmake/environment_hooks/sdf_path.bash"
)

# Install directories
install(DIRECTORY config launch fingrip fingrip/meshes fingrip/model resource resource/scene
  resource/renfort1
  resource/coude
  DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
