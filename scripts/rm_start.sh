#!/bin/bash
# RoboMaster 视觉系统启动脚本

# 加载环境
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash

# 设置日志目录
export RM_LOG_DIR=/home/thom/RM_version_ws/log
mkdir -p $RM_LOG_DIR

# 模式选择
MODE="${1:-auto}"

# 自动检测模式
if [ "$MODE" = "auto" ]; then
    if ls /dev/ttyACM* /dev/ttyUSB* &>/dev/null 2>&1; then
        echo "[INFO] 检测到串口设备，使用完整模式"
        MODE="full"
    else
        echo "[INFO] 未检测到串口设备，使用相机模式"
        MODE="camera"
    fi
fi

# 根据模式选择 launch 文件
case "$MODE" in
    full)
        LAUNCH_FILE="vision_bringup_can.launch.py"
        LOG_FILE="system_full.log"
        MODE_DESC="完整模式 (相机 + 外置CAN云台控制)"
        ;;
    camera)
        LAUNCH_FILE="camera_only.launch.py"
        LOG_FILE="system_camera.log"
        MODE_DESC="相机模式 (相机 + 检测 + 追踪，无串口)"
        ;;
    no_hardware)
        LAUNCH_FILE="no_hardware.launch.py"
        LOG_FILE="system_nohw.log"
        MODE_DESC="无硬件模式 (仅检测 + 追踪)"
        ;;
    *)
        echo "错误: 未知模式 '$MODE'"
        echo "使用方法: $0 [full|camera|no_hardware|auto]"
        echo "  full   - 完整模式 (需要串口设备)"
        echo "  camera - 相机模式 (无需串口)"
        echo "  no_hardware - 无硬件模式"
        echo "  auto   - 自动检测 (默认)"
        exit 1
        ;;
esac

# 启动系统
echo "================================================"
echo "[$(date)] 启动 RoboMaster 视觉系统"
echo "模式: $MODE_DESC"
echo "Launch 文件: $LAUNCH_FILE"
echo "================================================"
echo "[$(date)] 启动系统，模式: $MODE_DESC" >> $RM_LOG_DIR/$LOG_FILE

ros2 launch rm_vision_bringup $LAUNCH_FILE 2>&1 | tee -a $RM_LOG_DIR/$LOG_FILE
