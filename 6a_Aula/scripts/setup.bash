PACKAGE_DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd )"
FIRMWARE_DIR="$HOME/src/Firmware"
FIRMWARE_BUILD_DIR="$HOME/src/Firmware/build/px4_sitl_default"

pushd .

# Setup Gazebo to find this package's models and plugins
export GAZEBO_MODEL_PATH=${PACKAGE_DIR}/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=${PACKAGE_DIR}/../../devel/lib:${GAZEBO_PLUGIN_PATH}

# Setup Gazebo to find PX4
source ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_BUILD_DIR}

# Setup ROS to find the PX4 packages
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_DIR}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${FIRMWARE_DIR}/Tools/sitl_gazebo
echo -e "ROS_PACKAGE_PATH $ROS_PACKAGE_PATH"

popd