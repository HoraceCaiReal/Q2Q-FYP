#!/usr/bin/env bash
# Source this file in the Ubuntu VM before starting the step-3 stack or tools.
# Keep this environment identical for Gazebo plugins, ROS nodes and ROS CLI.
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset CYCLONEDDS_URI IGN_IP GZ_IP
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
# Project-local UDP transport avoids depending on stale shared-memory endpoints.
# Keep the same profile for Gazebo plugins, nodes and clients. No /dev/shm deletion.
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.q2q/fastdds_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$HOME/ros2_ws/install/robot_sim_control/lib${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
