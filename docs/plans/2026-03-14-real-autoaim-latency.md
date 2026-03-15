# 真实自瞄主链路延迟埋点实现计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 为主自瞄链路增加低频延迟统计日志，定位 `camera->detector`、`camera->tracker` 与 `camera->can_send` 三段真实控制延迟。

**Architecture:** 新增一个头文件级别的轻量统计窗口工具，用统一方式聚合最近一段时间的延迟样本；`armor_detector`、`armor_tracker`、`rm_can_bridge` 分别在本节点内采样并每秒打印一次统计。`rm_can_bridge` 额外只统计“新命令的首次发送”，避免把定时重复发送误当成真实端到端延迟。

**Tech Stack:** ROS 2 Humble, `rclcpp`, `ament_cmake`, `ament_cmake_auto`, `ament_cmake_gtest`, C++14/C++17

---

### Task 1: 提供可测试的延迟窗口工具

**Files:**
- Create: `rm_auto_aim/auto_aim_interfaces/include/auto_aim_interfaces/latency_window.hpp`
- Modify: `rm_auto_aim/auto_aim_interfaces/CMakeLists.txt`
- Create: `rm_auto_aim/auto_aim_interfaces/test/test_latency_window.cpp`

**Step 1: Write the failing test**
- 为 `LatencyWindow` 写 gtest，覆盖：
  - 连续样本的平均值/最大值/最后值统计
  - 未到周期前不产出快照
  - 到达周期后产出快照并重置窗口

**Step 2: Run test to verify it fails**
- Run: `colcon test --packages-select auto_aim_interfaces --event-handlers console_direct+ --ctest-args -R test_latency_window`
- Expected: 因缺少 `latency_window.hpp` 或目标未注册而失败

**Step 3: Write minimal implementation**
- 新增头文件 `LatencyWindow`
- 提供 `observe()`, `take_snapshot_if_due()` 等最小接口
- 在 `auto_aim_interfaces` 中安装 `include/` 并注册 gtest

**Step 4: Run test to verify it passes**
- Run: 同上
- Expected: `test_latency_window` 通过

### Task 2: 接入 `armor_detector` 与 `armor_tracker` 延迟统计

**Files:**
- Modify: `rm_auto_aim/armor_detector/src/detector_node.cpp`
- Modify: `rm_auto_aim/armor_detector/include/armor_detector/detector_node.hpp`
- Modify: `rm_auto_aim/armor_tracker/src/tracker_node.cpp`
- Modify: `rm_auto_aim/armor_tracker/include/armor_tracker/tracker_node.hpp`

**Step 1: Write/extend failing tests where practical**
- 若无需节点级复杂测试，则以 `LatencyWindow` 的已失败测试作为基础，并保持改动最小

**Step 2: Write minimal implementation**
- `armor_detector`：在现有单帧延迟计算位置接入窗口统计并每秒打印一次
- `armor_tracker`：在发布 `/tracker/target` 前计算并统计 `camera->tracker`
- 只在有效样本时记录，不改变原有消息流

**Step 3: Run targeted tests**
- Run: `colcon test --packages-select armor_detector auto_aim_interfaces --event-handlers console_direct+ --ctest-args -R 'test_latency_window|test_node_startup|test_detector_runtime_config|test_latest_frame_slot'`
- Expected: 相关测试通过

### Task 3: 为 `rm_can_bridge` 增加“首次发送”延迟统计

**Files:**
- Create: `rm_can_bridge/include/rm_can_bridge/pending_aim_latency.hpp`
- Create: `rm_can_bridge/test/test_pending_aim_latency.cpp`
- Modify: `rm_can_bridge/CMakeLists.txt`
- Modify: `rm_can_bridge/src/rm_can_bridge_node.cpp`

**Step 1: Write the failing test**
- 为 `PendingAimLatency` 写 gtest，覆盖：
  - 新命令只在第一次发送时返回延迟样本
  - 同一命令后续重复发送不重复返回样本
  - 新命令到来后可再次统计

**Step 2: Run test to verify it fails**
- Run: `colcon test --packages-select rm_can_bridge --event-handlers console_direct+ --ctest-args -R test_pending_aim_latency`
- Expected: 因缺少头文件或符号而失败

**Step 3: Write minimal implementation**
- 新增 `PendingAimLatency` 小工具
- 在 `targetCallback()` / `armorsCallback()` 成功更新命令后登记源时间戳
- 在 `sendGimbalFrame()` 首次发送该命令时记录 `camera->can_send`
- 保持原有 CAN 帧格式和发送逻辑不变

**Step 4: Run test to verify it passes**
- Run: 同上
- Expected: `test_pending_aim_latency` 通过

### Task 4: 运行定向验证与实机检查

**Files:**
- No code changes expected

**Step 1: Run package tests**
- Run: `colcon test --packages-select auto_aim_interfaces armor_detector rm_can_bridge --event-handlers console_direct+`
- Expected: 相关包测试通过

**Step 2: Inspect test results**
- Run: `colcon test-result --verbose`
- Expected: 新增测试通过，未引入相关回归失败

**Step 3: Run runtime validation**
- Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch rm_vision_bringup vision_bringup_can.launch.py`
- Expected: 日志中出现：
  - `detector latency: ...`
  - `tracker latency: ...`
  - `can_send latency: ...`

**Step 4: Observe and rank the bottleneck**
- 若 `detector` 最大：下一轮优先优化相机抓帧/像素转换/检测热路径
- 若 `can_send` 最大：下一轮优先检查 `absolute_from_feedback`、反馈新鲜度和发送节奏

