#!/bin/bash
# RoboMaster 视觉系统调试脚本

source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash

export RM_LOG_DIR=/home/thom/RM_version_ws/log
mkdir -p $RM_LOG_DIR

echo "[$(date)] 启动调试模式..." >> $RM_LOG_DIR/debug.log
# 启动 Foxglove Bridge 用于可视化
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 2>&1 | tee -a $RM_LOG_DIR/debug.log
