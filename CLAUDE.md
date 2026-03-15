# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是 RoboMaster (RM) 机器人的 ROS 2 Humble 开发环境。项目采用 Docker 容器化开发，实际源代码在容器构建时从外部仓库克隆。

## 核心组件

项目由以下 ROS 2 包组成：

- **rm_auto_aim** - 自动瞄准系统，识别并打击目标
- **ros2_mindvision_camera** - MindVision 工业相机驱动
- **ros2_hik_camera** - 海康威视工业相机驱动
- **rm_gimbal_description** - 云台机械结构描述（URDF/Mesh）
- **rm_serial_driver** - 下位机串口通信驱动
- **rm_vision** - 视觉处理核心系统

## Docker 部署指南

### 前置准备

#### 1. 卸载旧版 Docker（如已安装）
部分 mini PC 预装的 Docker 版本可能过老，建议先卸载：

```bash
# 卸载旧版本
sudo apt-get purge docker-ce docker-ce-cli containerd.io
```

#### 2. 安装最新版 Docker
```bash
curl -fsSL https://get.docker.com | bash -s docker --mirror Aliyun
```

**重要：在选择选项时输入序号 8！**

安装时间约 5-8 分钟，完成后验证：
```bash
docker version
```

#### 3. （选装）Docker 图形化工具
可安装 Portainer 或其他 Docker GUI 工具以获得更便捷的管理体验。

### 构建镜像

**提示**：如果有团队已构建好的镜像，建议直接 `docker pull`，无需自行构建。

建议修改 **Dockerfile** 中的仓库链接为本队的开发仓库链接。

```bash
# 在 Dockerfile 所在目录执行
sudo docker build -t rm_vision01 .
```

- `rm_vision01` 为镜像名称，可自定义
- 构建时间约半小时

### 创建容器

#### 开发容器 (rv_devel)
用于代码开发和调试，启动 foxglove_bridge 发布数据：

```bash
docker run -it --name rv_devel \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v $HOME/.ros:/root/.ros \
  -v ws:/ros_ws \
  rm_vision01:latest
```

#### 运行容器 (rv_runtime)
用于实际运行 rm_vision 系统：

```bash
docker run -it --name rv_runtime \
  --privileged \
  --network host \
  --restart always \
  -v /dev:/dev \
  -v $HOME/.ros:/root/.ros \
  -v ws:/ros_ws \
  rm_vision01:latest
```

**参数说明**：
- `--privileged` - 特权模式，访问硬件设备
- `--network host` - 使用宿主机网络
- `--restart always` - 容器意外退出时自动重启（仅运行容器）
- `-v /dev:/dev` - 挂载设备文件（相机、串口等）
- `-v $HOME/.ros:/root/.ros` - 挂载 ROS 日志和配置
- `-v ws:/ros_ws` - 挂载工作空间（持久化）

### 容器管理

```bash
# 查看所有容器
docker ps -a

# 启动容器
docker start <container_id>

# 进入容器
docker attach <container_id>

# 停止容器
docker stop <container_id>

# 删除容器
docker rm <container_id>
```

## ROS 2 运行命令

### 进入容器后的操作

**故障排查提示**：如果遇到找不到文件等奇怪错误，重启容器后执行 `source install/setup.bash` 可解决大部分问题。

#### 在运行容器 (rv_runtime)

```bash
# 1. 启动容器
docker start <container_id>

# 2. 进入容器
docker attach <container_id>

# 3. 运行 rm_vision 视觉系统
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

#### 在开发容器 (rv_devel)

```bash
# 1. 启动容器
docker start <container_id>

# 2. 进入容器
docker attach <container_id>

# 3. 启动 Foxglove Bridge（发布数据用于可视化）
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### 开发时重建工作空间

```bash
# 在容器内执行
cd /ros_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.zsh
```

### 构建特定包

```bash
# 只构建特定包
colcon build --packages-select <package_name> --symlink-install

# 并行构建多个包
colcon build --packages-select <package1> <package2> --symlink-install

# 构建时指定编译器（如需）
colcon build --symlink-install --cmake-args "-DCMAKE_CXX_COMPILER=/usr/bin/clang++"
```

### 运行单个节点

```bash
# 运行特定节点
ros2 run <package_name> <executable_name>

# 运行相机驱动
ros2 run hik_camera hik_camera_node

# 运行装甲板检测器
ros2 run armor_detector armor_detector_node
```

### 常用 ROS 2 命令

```bash
# 查看话题列表
ros2 topic list

# 查看话题内容
ros2 topic echo /topic_name

# 查看节点列表
ros2 node list

# 查看节点信息
ros2 node info /node_name
```

### 单元测试
```bash
# 运行特定包的测试
colcon test --packages-select <package_name>

# 运行所有包的测试
colcon test --packages-select rm_vision rm_auto_aim ros2_hik_camera rm_serial_driver

# 运行测试并显示详细输出
colcon test --event-handlers console_direct+

# 查看测试结果
colcon test-result --all

# 查看特定包的测试结果
colcon test-result --packages-select <package_name>
```

## 系统架构

```
相机驱动 (mindvision/hik) → rm_vision (视觉处理) → rm_auto_aim (自动瞄准)
                                                          ↓
                                                    rm_serial_driver
                                                          ↓
                                                        下位机
```

- **相机驱动** 发布原始图像到 `/camera/image_raw`
- **rm_vision** 订阅图像，处理后发布目标信息
- **rm_auto_aim** 订阅目标信息和云台状态，计算瞄准角度
- **rm_serial_driver** 与下位机通信，发送控制指令并接收状态反馈

## Foxglove 可视化

项目已集成 `foxglove-bridge`，支持实时可视化调试。

### 使用方式

1. **在开发容器启动 Foxglove Bridge**：
   ```bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
   ```

2. **使用 Foxglove Studio 连接**：
   - 打开 [Foxglove Studio](https://studio.foxglove.dev/) 或安装桌面版
   - 添加新连接 → 选择 **Foxglove Bridge**
   - 输入地址：`ws://<主机IP>:8765`（本地用 `ws://localhost:8765`）

### 可视化内容

- 📷 相机图像流（原始图像、处理后图像）
- 🎯 目标检测框和追踪信息
- 📐 云台姿态数据
- 📊 调试信息和性能指标

**默认端口**：`8765`

## 注意事项与技巧

### 环境配置
- **ROS 2 发行版**：Humble
- **Shell**：容器内使用 zsh，已配置自动建议和语法高亮
- **构建选项**：推荐使用 `--symlink-install`，方便 Python 脚本开发时实时生效

### 开发建议
- 安装 **VSCode Docker 插件** 可获得更舒适的开发体验
- 相机和串口设备需要通过 `--device` 参数或 `-v /dev:/dev` 挂载
- 修改代码后记得重新 `colcon build` 并 `source install/setup.zsh`

### 故障排查
| 问题 | 解决方案 |
|------|----------|
| 找不到文件/模块 | 重启容器，执行 `source /ros_ws/install/setup.zsh` |
| 构建镜像失败 | 确认 Docker 版本是否过旧，建议重新安装最新版 |
| 无法访问相机/串口 | 检查是否使用 `--privileged` 和 `-v /dev:/dev` |
| Foxglove 无法连接 | 确认防火墙设置，端口 8765 是否开放 |
| 编译错误 | 检查 C++ 标准设置（项目使用 C++14），确保使用 `--symlink-install` |
| 依赖问题 | 运行 `rosdep update && rosdep install --from-paths src --ignore-src -r -y` |

### 调试技巧
- **日志级别控制**：通过 launch 文件中的参数控制各节点日志级别
- **实时调试**：使用 `ros2 topic echo` 监听话题数据流
- **可视化调试**：Foxglove Bridge 支持实时查看图像和处理结果

### 代码仓库
主要源代码托管在 Gitee：
- 仓库前缀：`https://gitee.com/LihanChen2004/`
- 建议修改 Dockerfile 中的仓库链接为本队开发仓库
- 容器构建时使用 `--depth=1` 浅克隆

### 依赖仓库关系
项目包含以下主要子模块：
- **rm_auto_aim** - 自动瞄准系统（包含 armor_detector 和 armor_tracker）
- **ros2_mindvision_camera** - MindVision 相机驱动
- **ros2_hik_camera** - 海康威视相机驱动
- **rm_gimbal_description** - 云台 URDF 描述文件
- **rm_serial_driver** - 串口通信驱动
- **rm_vision** - 视觉处理总线和启动文件
- **auto_aim_interfaces** - 自定义接口定义

## 原生部署

### 环境要求
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.11+
- GCC 11.4+

### 快速部署

```bash
# 1. 安装依赖
sudo apt update
sudo apt install -y build-essential python3-rosdep2 python3-colcon
sudo apt install -y ros-humble-rosidl-default-generators ros-humble-vision-opencv
sudo apt install -y ros-humble-cv-bridge ros-humble-foxglove-bridge

# 2. 安装 ROS 2 依赖
cd /home/thom/RM_version_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. 编译工作空间
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. 配置权限
sudo usermod -a -G video,dialout $USER
sudo reboot  # 需要重启生效

# 5. 启动系统
source install/setup.bash
./scripts/rm_start.sh

# 调试模式
./scripts/rm_debug.sh
```

### 启动脚本

- `./scripts/rm_start.sh` - 启动主系统
- `./scripts/rm_debug.sh` - 启动调试模式（带 Foxglove）
- `./scripts/rm_camera_test.sh [hik|mv]` - 测试相机
- `./scripts/rm_reset.sh` - 重置工作空间

## 沟通原则
- 与用户的所有回复与沟通，文档与代码注释均使用中文，必要时可保留英文专业名词，并在首次出现时附简要中文注释。