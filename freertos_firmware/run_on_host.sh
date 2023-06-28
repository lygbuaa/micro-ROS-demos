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

# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &

# Use RMW Micro XRCE-DDS implementation
export RMW_IMPLEMENTATION=rmw_microxrcedds
# ros2 run pseudo_pnc planning &
# ros2 run pseudo_pnc control
ros2 run pseudo_pnc pnc

# kill -15 `ps -e | grep "control" | awk '{print $1}'`
kill -15 `ps -e | grep "planning" | awk '{print $1}'`
kill -15 `ps -e | grep "agent" | awk '{print $1}'`