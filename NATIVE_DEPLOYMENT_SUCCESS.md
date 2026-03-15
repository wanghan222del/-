# 原生部署成功！

## 🎉 部署完成状态

### ✅ 已完成的所有任务
1. ✅ 安装系统依赖
2. ✅ 创建启动脚本
3. ✅ ROS 2 依赖配置
4. ✅ 编译工作空间
5. ✅ 系统权限配置
6. ✅ 创建配置文件
7. ✅ 更新文档
8. ✅ 系统验证测试

## 📦 编译状态
- ✅ **成功编译**：rm_vision_bringup, hik_camera, rm_gimbal_description, rm_serial_driver, auto_aim_interfaces, armor_detector, armor_tracker
- ✅ **所有核心包编译完成！**

## 🚀 快速启动

### 1. 启动系统（多模式支持）
```bash
# 加载环境
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash

# 自动模式（推荐）
# 检测串口设备自动选择：有串口→完整模式，无串口→相机模式
./scripts/rm_start.sh

# 完整模式（相机 + 串口 + 检测 + 追踪）
# 需要连接下位机串口设备
./scripts/rm_start.sh full

# 相机模式（相机 + 检测 + 追踪，无串口）
# 适用于没有下位机的开发调试场景
./scripts/rm_start.sh camera

# 无硬件模式（仅检测 + 追踪算法）
./scripts/rm_start.sh no_hardware
```

### 2. 调试模式（带 Foxglove 可视化）
```bash
# 启动 Foxglove Bridge 用于可视化
./scripts/rm_debug.sh
```

## 🎯 仅相机测试模式

适用于**只有相机没有下位机**的开发调试场景。

### 功能特性
- ✅ 相机图像采集
- ✅ 装甲板检测
- ✅ 装甲板追踪
- ✅ 目标瞄准计算
- ✅ TF 坐标变换（静态）
- ❌ 云台控制（需要下位机）
- ❌ 下位机通信

### 使用场景
1. **算法开发**：在没有机器人硬件的情况下测试视觉算法
2. **功能验证**：验证相机、检测器和追踪器的独立功能
3. **参数调试**：调整检测和追踪参数而不影响机器人

### 可视化调试

使用 Foxglove Studio 查看实时数据：
```bash
# 终端 1：启动相机模式
./scripts/rm_start.sh camera

# 终端 2：启动 Foxglove Bridge
./scripts/rm_debug.sh
```

然后在 Foxglove Studio 中添加面板：
- **Image**：`/camera/image_raw` - 相机图像流
- **Marker**：`/detector/marker` - 检测框和置信度
- **Marker**：`/tracker/marker` - 追踪目标
- **TF**：查看坐标系变换树

连接地址：`ws://localhost:8765`

### 技术说明
相机模式使用 `static_transform_publisher` 提供固定的 TF 变换（`odom` → `gimbal_link`），替代串口驱动的动态变换。这意味着：
- 云台姿态固定为零位（roll=0, pitch=0, yaw=0）
- 追踪坐标基于固定的相机姿态
- 无法反映真实的云台运动

### 3. 相机单独测试
```bash
# 测试海康威视相机
./scripts/rm_camera_test.sh hik

# 测试 MindVision 相机
./scripts/rm_camera_test.sh mv
```

### 4. 工作空间重置
```bash
# 清理并重新编译
./scripts/rm_reset.sh
```

## 🔗 Foxglove 可视化
- **连接地址**：`ws://localhost:8765`
- **打开方式**：
  1. 打开 [Foxglove Studio](https://studio.foxglove.dev/)
  2. 添加新连接 → 选择 **Foxglove Bridge**
  3. 输入地址：`ws://localhost:8765`

## 📁 重要文件
- `scripts/rm_start.sh` - 主系统启动脚本
- `scripts/rm_debug.sh` - 调试模式脚本
- `scripts/rm_camera_test.sh` - 相机测试脚本
- `scripts/rm_reset.sh` - 工作空间重置脚本
- `config/camera_config.yaml` - 相机配置
- `config/serial_config.yaml` - 串口配置

## ⚠️ 注意事项
- ✅ 所有装甲板检测和追踪功能现已编译成功！
- **相机模式**使用海康威视工业相机（USB 3.0 Vision 协议），不通过 /dev/video* 接口
- 串口通信需要设备（/dev/ttyACM* 或 /dev/ttyUSB*）才能正常工作
- 已安装必要的 curl 和 GDAL 库支持

### 相机配置说明
如果遇到相机参数错误（如 "exposure_time doesn't comply with integer range"），说明当前值超出了相机支持的范围。

**解决方法**：
1. 查看相机支持的参数范围
2. 编辑配置文件：`rm_vision/rm_vision_bringup/config/node_params.yaml`
3. 调整 `exposure_time` 和 `gain` 参数到有效范围

**示例参数**（海康威视 MV-CS016-10UC）：
```yaml
/camera_node:
  ros__parameters:
    camera_info_url: package://rm_vision_bringup/config/camera_info.yaml
    exposure_time: 5000   # 曝光时间（微秒），根据相机范围调整
    gain: 8.0             # 增益值（dB），根据相机范围调整
```

**获取相机参数范围**：
```bash
# 查看节点日志中显示的参数范围
ros2 run hik_camera hik_camera_node
```

## 📋 验证清单
- [x] 系统依赖已安装
- [x] 启动脚本语法正确
- [x] ROS 2 包可正常加载
- [x] 环境变量已配置
- [x] 配置文件已创建
- [x] 文档已更新
