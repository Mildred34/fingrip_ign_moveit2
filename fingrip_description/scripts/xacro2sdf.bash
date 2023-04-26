#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `fingrip_description` package
# This script is called in fingrip's package CMakelist.txt

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/fingrip.urdf.xacro"
SDF_PATH="$(dirname "${SCRIPT_DIR}")/fingrip/model.sdf"
TMP_URDF_PATH="/tmp/fingrip_tmp.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=fingrip
    gripper:=true
    collision_arm:=true
    collision_gripper:=true
    ros2_control:=true
    ros2_control_plugin:=ign
    ros2_control_command_interface:=effort
    gazebo_preserve_fixed_joint:=false
    underactuation:=true
    abduction:=true
    longFingertip:=false
    nbFingers:=3
    hand_type:=1
)

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
gz sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/fingrip_description\///g" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null
