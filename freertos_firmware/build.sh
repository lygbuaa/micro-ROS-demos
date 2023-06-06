#!/bin/bash

function find_uros_ws_path() {
    # echo "@i@ --> find dir: ${0}"
    this_script_dir=$( dirname -- "$0"; )
    pwd_dir=$( pwd; )
    ros2_ws_path=${pwd_dir}"/"${this_script_dir}"/../../../../"
    echo "${ros2_ws_path}"
}

# ROS2_WS_PATH=/home/hugoliu/github/microros_ws/
UROS_WS_PATH=$( find_uros_ws_path )

source /opt/ros/$ROS_DISTRO/setup.bash
source ${UROS_WS_PATH}/install/setup.bash

cd ${UROS_WS_PATH}
ros2 run micro_ros_setup build_firmware.sh