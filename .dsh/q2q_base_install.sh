#!/usr/bin/env bash
# q2q_base_install.sh - install base assets for the Q2Q FYP demo
# Target: Ubuntu 22.04 + ROS2 Humble + Gazebo Fortress + Clearpath (Husky A200 + Kinova Gen3 Lite + 2F Lite)
# Idempotent; run as:  sudo bash q2q_base_install.sh
set -euo pipefail

log() { echo -e "\n\033[1;34m== $*\033[0m"; }

log "apt update"
sudo apt update

log "install Gazebo Fortress (ignition-fortress -> gz-sim6)"
sudo apt install -y ignition-fortress

log "install ros_gz bridge + gz ros2_control"
sudo apt install -y ros-humble-ros-gz ros-humble-gz-ros2-control

log "install Nav2"
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

log "install MoveIt2"
sudo apt install -y ros-humble-moveit

log "install ros2_control + controllers"
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

log "install Clearpath simulator + robot descriptions (Husky + Kinova)"
sudo apt install -y \
  ros-humble-clearpath-simulator \
  ros-humble-clearpath-desktop \
  ros-humble-clearpath-description \
  ros-humble-clearpath-manipulators-description

log "DONE. Verify with:  gz sim --version   and   ros2 pkg list | grep -E 'clearpath|nav2|moveit|gz'"
