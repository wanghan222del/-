#!/bin/bash
# 相机单独测试脚本

source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash

# 根据配置启动相机
case "${1:-hik}" in
    "hik")
        ros2 run hik_camera hik_camera_node
        ;;
    "mv")
        ros2 run mindvision_camera MVCameraNode
        ;;
    *)
        echo "使用方法: $0 [hik|mv]"
        exit 1
        ;;
esac
