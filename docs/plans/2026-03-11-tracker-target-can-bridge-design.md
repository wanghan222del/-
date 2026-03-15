# Tracker Target CAN Bridge Design

## Goal

在不降低相机检测与追踪帧率的前提下，保留 `rm_vision` 现有的相机、检测、追踪主链路，替换最后一跳控制输出，使系统通过当前 USB-CAN 串口持续发送云台相对瞄准角 `yaw/pitch`，并在同一 `rm_can_bridge` 节点内接收 `0x402` 云台反馈角度。

## Constraints

- 上游自瞄链路保持一致：`camera -> armor_detector -> armor_tracker -> /tracker/target`
- 不继续使用 `rm_serial_driver` 作为云台角度发送链路
- `vision_bringup.launch.py` 原样不可用，因为它会启动 `rm_serial_driver` 抢占 `/dev/ttyACM0`
- 要保证图像链路帧率，不允许引入额外图像处理或高频重日志
- 需要保留一个用户可调配置文件，用于修正相机与发射管不重合带来的瞄准误差
- 发送侧 `0x301` 继续按 **deg 语义的 q10.6** 发给云台
- 接收侧 `0x402` 反馈按 **q10.6** 解码，并在 ROS 中以 **deg 浮点值** 对外发布
- 收发逻辑统一放在 `rm_can_bridge` 内，避免多个进程抢占同一 USB-CAN 串口

## Accepted Architecture

### Phase 1

第一阶段采用“追踪结果直发 CAN”的轻量方案：

1. 保留 `vision_bringup` 的相机、检测、追踪主结构
2. 新增 `vision_bringup_can.launch.py`，用它替代原始 `vision_bringup.launch.py`
3. 在新 launch 中移除 `rm_serial_driver`
4. 在新 launch 中启动扩展后的 `rm_can_bridge`
5. 将 `armor_tracker` 的 `target_frame` 临时改为 `gimbal_link`
6. `rm_can_bridge` 新增 `tracker_target` 输入模式，直接订阅 `/tracker/target`
7. 同一个 `rm_can_bridge` 同时负责发送 `0x301` 和接收 `0x402`

这样做后，`/tracker/target` 中的目标位置已经位于 `gimbal_link` 坐标系，无需在桥接节点中先依赖实时云台 TF，即可先跑通“持续追踪 -> 相对角度 -> CAN 发送”的链路。

### Phase 2

后续在已经具备 `0x402` 接收能力的基础上，再把收到的 `yaw/pitch` 用于更新 TF；届时可将 `armor_tracker.target_frame` 切回 `odom`，恢复更标准的惯性系追踪与控制闭环。

## Why Not Reuse `vision_bringup.launch.py` Directly

原始 `vision_bringup.launch.py` 会启动 `rm_serial_driver`，而它默认也会打开 `/dev/ttyACM0`。当前 USB-CAN 设备同样占用 `/dev/ttyACM0`，若继续原样使用，会导致原串口驱动与 CAN 桥抢占同一设备节点，且双方使用的协议完全不同。因此需要新增 `vision_bringup_can.launch.py`，而不是直接复用原始 launch。

## Data Flow

### Existing upstream chain

`Hik/MV camera -> armor_detector -> armor_tracker -> /tracker/target`

### New downstream chain

`/tracker/target -> rm_can_bridge (tracker_target mode) -> /dev/ttyACM0 -> USB-CAN -> 0x301 yaw/pitch`

### Feedback chain

`USB-CAN -> /dev/ttyACM0 -> rm_can_bridge -> parse 0x402 q10.6 -> ROS deg feedback topics`

### Loss behavior

- 当 `tracking == true` 且目标未超时时，持续发送相对瞄准角
- 当 `tracking == false` 或目标超时时：
  - `0x301` 清零
  - 不保持上一帧角度
- `0x311` 暂保持现有行为，不作为本次改造核心

## Relative Aim Computation

### Input frame

第一阶段中，`/tracker/target.header.frame_id` 预期为 `gimbal_link`。

### User-tunable offset

新增配置参数：

- `muzzle_offset_xyz_in_gimbal: [x, y, z]`

含义：发射管原点相对于 `gimbal_link` 的三维偏移，单位米。

这样设计的原因：

- 用户当前主要需要修正相机与发射管高度差，常改的是 `z`
- 但后续若发现前后或左右也有偏，不需要再改代码，直接改同一组 `x/y/z`

### Geometry

给定 `target.position = (x_t, y_t, z_t)`（位于 `gimbal_link`）以及 `muzzle_offset = (x_m, y_m, z_m)`，先求相对枪口的目标向量：

- `dx = x_t - x_m`
- `dy = y_t - y_m`
- `dz = z_t - z_m`

采用 REP-103 坐标语义（`gimbal_link`：x 前、y 左、z 上）计算相对瞄准角：

- `yaw = atan2(dy, dx)`
- `pitch = atan2(dz, hypot(dx, dy))`

随后按当前 `rm_can_bridge` 已有的 `q10.6` 编码与 `0x301` 封包逻辑发送。发送语义固定为 **deg**。

## CAN Protocol Mapping

### 0x301 Send

- 继续给云台发送相对瞄准角
- `data[0..1]`：yaw（deg，q10.6，大端）
- `data[2..3]`：pitch（deg，q10.6，大端）
- 其余字段保持当前桥接定义

### 0x402 Receive

接收的 8 字节 payload 定义如下，均为 **q10.6 16 位定点数，大端**：

- `data[0..1]`：yaw
- `data[2..3]`：pitch
- `data[4..5]`：roll
- `data[6..7]`：yaw_motor_pos

这些值来自 STM32 侧的 `q_10_6_t` 语义，参考：

- `/home/thom/下载/q_10_6_t.h`
- `/home/thom/下载/q_10_6_t.cpp`

桥接节点在字节级解包后，立即按 `q10.6 -> deg 浮点` 解码，不把这些量作为“普通 int 角度”使用。

## Feedback ROS Interface

为节省时间并避免新增消息包，首版使用标准消息：

- `/gimbal/feedback/rpy_deg`
  - 类型：`geometry_msgs/msg/Vector3Stamped`
  - 语义：`x = roll_deg`, `y = pitch_deg`, `z = yaw_deg`
- `/gimbal/feedback/yaw_motor_pos_deg`
  - 类型：`std_msgs/msg/Float64`

后续如需要更强语义，可再换成自定义消息。

## Configuration

新增配置文件：`rm_can_bridge/config/tracker_target_bridge.yaml`

建议首版包含：

- `input_mode: tracker_target`
- `target_topic: /tracker/target`
- `serial_port: /dev/ttyACM0`
- `serial_baud: 921600`
- `angle_unit: deg`
- `target_timeout_ms: 100`
- `zero_when_lost: true`
- `muzzle_offset_xyz_in_gimbal: [0.0, 0.0, 0.0]`
- `feedback_can_id: 0x402`
- `publish_feedback: true`

该文件作为用户后续调枪口补偿的唯一常用入口。

## Launch Design

新增 `rm_vision/rm_vision_bringup/launch/vision_bringup_can.launch.py`：

- 保留 `robot_state_publisher`
- 保留相机 + detector 容器
- 保留 `armor_tracker`
- 对 `armor_tracker` 参数覆盖：`target_frame = gimbal_link`
- 删除 `rm_serial_driver`
- 增加 `rm_can_bridge` 节点，加载 `tracker_target_bridge.yaml`

该 launch 的职责是成为新的“一条命令启动完整视觉 + CAN 输出”的入口。

## Performance Strategy

本设计对帧率的影响被控制在最小范围：

- 不改图像采集节点
- 不改 detector 推理逻辑
- 不新增图像话题订阅者
- 不在 detector 容器中加入 CAN 逻辑
- 仅在 `/tracker/target` 上增加一个轻量订阅者
- CAN 接收解析只处理固定长度的 30 字节 USB-CAN 帧，不引入额外图像计算
- 每帧只做常数级几何计算与已有串口封包发送

若实现后出现帧率回退，优先检查：

1. CAN 节点日志级别是否过高
2. 是否在高频回调中执行阻塞 I/O
3. 是否错误地把桥接逻辑塞进图像组件容器

## Verification Plan

### Functional verification

1. `vision_bringup_can.launch.py` 能正常拉起相机、检测、追踪与 CAN 桥
2. `/tracker/target` 正常发布，且 `armor_tracker.target_frame == gimbal_link`
3. 手动向 `/tracker/target` 注入测试目标时，CAN 分析仪上 `0x301` 随目标位置变化
4. 丢目标或超时时，`0x301` 清零
5. 当 CAN 分析仪或下位机返回 `0x402` 时，`/gimbal/feedback/rpy_deg` 与 `/gimbal/feedback/yaw_motor_pos_deg` 正常发布且单位为 deg

### Performance verification

1. `/image_raw` 帧率与改造前保持同一量级
2. `/tracker/target` 发布频率无明显下降
3. CAN 发送/接收同时开启后，CPU 不出现异常飙升

## Files Expected To Change

- Modify: `rm_can_bridge/src/rm_can_bridge_node.cpp`
- Modify: `rm_can_bridge/launch/rm_can_bridge.launch.py`
- Modify: `rm_can_bridge/CMakeLists.txt`
- Modify: `rm_can_bridge/package.xml`
- Create: `rm_can_bridge/config/tracker_target_bridge.yaml`
- Create: `rm_can_bridge/include/rm_can_bridge/target_aim_math.hpp`
- Create: `rm_can_bridge/include/rm_can_bridge/q10_6_angle.hpp`
- Create: `rm_can_bridge/test/test_target_aim_math.cpp`
- Create: `rm_can_bridge/test/test_q10_6_angle.cpp`
- Modify: `rm_vision/rm_vision_bringup/launch/common.py`
- Create: `rm_vision/rm_vision_bringup/launch/vision_bringup_can.launch.py`

## Non-Goals In This Phase

- 不实现完整下位机闭环控制器
- 不恢复 `odom` 惯性系闭环
- 不重写 detector 或 tracker 算法
- 不改变现有 `0x301` CAN 格式
- 不修改下位机协议
