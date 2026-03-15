# 真实自瞄主链路延迟埋点设计

**目标**

在不改变主自瞄控制逻辑的前提下，量出真实主链路的端到端延迟，定位拖延感主要来自哪一段：
- `camera_node -> armor_detector`
- `camera_node -> armor_tracker`
- `camera_node -> rm_can_bridge` 首次发送云台 CAN 指令

**问题背景**

当前用户在 Foxglove 中观察到明显画面滞后，但 Foxglove 本身会引入图像传输、解码与渲染延迟，不能作为主自瞄控制延迟的真值。现有系统虽然已经有 `armor_detector` 内部的单点延迟计算，但没有形成完整主链路的统一统计，也无法回答延迟主要堆积在哪一层。

**方案选择**

- 方案 A（采用）：在主链路节点内增加低频节流统计日志，统一沿用现有消息头时间戳，不新增话题、不改消息定义。
- 方案 B：新增专用 latency debug 话题，将整条链路的延迟状态发布出去。

选择方案 A，因为它改动最小、对现有接口零侵入、最适合当前“先定位瓶颈再优化”的目标。

**设计原则**

1. 不把 Foxglove 观感当成真实控制延迟。
2. 不修改 `Armors`/`Target` 等现有消息定义。
3. 不改变检测、跟踪、CAN 发送的功能逻辑。
4. 统计日志必须低频，避免日志本身反向拖慢系统。
5. `camera->can_send` 只统计“新目标/新命令进入后第一次实际发送”的延迟，避免对同一条旧命令重复累计。

**数据流与埋点口径**

### 1. `camera->detector`

- 起点：`sensor_msgs::msg::Image::header.stamp`
- 终点：`armor_detector` 完成当前帧检测、准备发布检测结果时的 `now()`
- 位置：`rm_auto_aim/armor_detector/src/detector_node.cpp`

### 2. `camera->tracker`

- 起点：`Armors.header.stamp`（该时间戳本身来自图像时间戳）
- 终点：`armor_tracker` 发布 `/tracker/target` 前的 `now()`
- 位置：`rm_auto_aim/armor_tracker/src/tracker_node.cpp`

### 3. `camera->can_send`

- 起点：`Target.header.stamp`（或 `Armors.header.stamp`，取决于 `rm_can_bridge` 输入模式）
- 终点：`rm_can_bridge` 真正发送 `0x301` 云台指令前的 `now()`
- 注意：只统计收到新目标后“第一次成功发送”的延迟，不对后续定时重复发送的同一命令反复统计
- 位置：`rm_can_bridge/src/rm_can_bridge_node.cpp`

**实现细节**

### 延迟统计窗口

引入一个轻量级、可复用的头文件工具，负责：
- 记录最近一次延迟
- 累积窗口内样本数
- 计算窗口平均值
- 记录窗口最大值
- 到达统计周期后输出快照并清空窗口

该工具放在 `auto_aim_interfaces/include/auto_aim_interfaces/latency_window.hpp`，供 `armor_detector`、`armor_tracker`、`rm_can_bridge` 共同使用。

### `rm_can_bridge` 的首次发送判定

新增极小状态：
- 最近一次成功生成瞄准命令对应的源时间戳
- 一个“待统计首次发送”标记

当 `targetCallback()` / `armorsCallback()` 成功更新瞄准命令后，标记“有一条新的待发送命令”；`sendGimbalFrame()` 在第一次成功发出对应命令时记录一次 `camera->can_send`，随后清除该标记。

**日志形式**

每秒输出一次，格式类似：
- `detector latency: samples=... avg=...ms max=...ms last=...ms`
- `tracker latency: samples=... avg=...ms max=...ms last=...ms`
- `can_send latency: samples=... avg=...ms max=...ms last=...ms`

**错误处理**

- 消息头时间戳无效或为零：跳过当前样本，不报高频错误
- `rm_can_bridge` 当前没有新命令、或者命令因反馈不新鲜未生效：不记录 `camera->can_send`
- 统计窗口在当前周期没有样本：不输出空日志

**测试策略**

1. 为 `LatencyWindow` 写纯单元测试，验证：
   - 窗口内统计正确
   - 达到周期后产生快照
   - 输出快照后窗口被重置
2. 为 `rm_can_bridge` 的“首次发送统计”写纯单元测试，验证：
   - 新命令只在第一次发送时产生日志样本
   - 连续发送同一命令不会重复记账
   - 新命令到来后可再次统计
3. 跑 `armor_detector` 与 `rm_can_bridge` 定向测试
4. 最后运行系统实机启动，观察三段延迟日志是否稳定输出

**验收标准**

满足以下条件则认为本轮工作完成：
1. `vision_bringup_can.launch.py` 启动后，日志中可稳定看到 `detector`、`tracker`、`can_send` 三段延迟统计
2. 统计值明显基于真实消息链路时间戳，而不是 Foxglove 渲染时间
3. 不影响 `/tracker/target` 与 `rm_can_bridge` 现有控制逻辑
4. 新增单元测试通过
