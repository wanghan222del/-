# rm_can_bridge USB-CAN 双格式接收实现计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 让 `rm_can_bridge` 同时解析旧 `30` 字节 USB-CAN 帧和新 `16` 字节 USB-CAN 帧，并继续只发布 `0x402` 的 8 字节反馈数据。

**Architecture:** 把串口帧识别提取到独立头文件解析器中，节点只负责消费缓冲区和按 `feedback_can_id` 过滤。发送 `0x301` 的逻辑不改，确保与下位机现有协议完全一致。

**Tech Stack:** ROS 2, `rclcpp`, `ament_cmake_gtest`, C++17

---

### Task 1: 让失败测试真正跑起来

**Files:**
- Create: `rm_can_bridge/include/rm_can_bridge/usb_can_frame_parser.hpp`
- Test: `rm_can_bridge/test/test_usb_can_frame_parser.cpp`

**Step 1: Write the failing test**
- 现有 `test_usb_can_frame_parser.cpp` 已经覆盖旧/新两种帧格式和异常分支

**Step 2: Run test to verify it fails**
- Run: `colcon test --packages-select rm_can_bridge --event-handlers console_direct+ --ctest-args -R test_usb_can_frame_parser`
- Expected: 因缺少 `usb_can_frame_parser.hpp` 或符号未定义而失败

### Task 2: 实现最小解析器

**Files:**
- Create: `rm_can_bridge/include/rm_can_bridge/usb_can_frame_parser.hpp`

**Step 1: Write minimal implementation**
- 定义 `ParsedUsbCanFrame`
- 定义 `UsbCanParseAction`
- 实现 `tryParseUsbCanFrame()`
- 兼容：
  - `55 AA 1E ... 88` 旧格式
  - `AA xx 08 id0 id1 id2 id3 payload[8] 55` 新格式

**Step 2: Run test to verify it passes**
- Run: 同上
- Expected: `test_usb_can_frame_parser` 通过

### Task 3: 接入节点接收循环

**Files:**
- Modify: `rm_can_bridge/src/rm_can_bridge_node.cpp`

**Step 1: Replace direct frame parsing**
- 让 `consumeSerialFrames()` 改为调用解析器
- 解析成功后统一走现有反馈发布逻辑
- 保留现有调试日志风格

**Step 2: Run package tests**
- Run: `colcon test --packages-select rm_can_bridge --event-handlers console_direct+`
- Expected: `rm_can_bridge` 相关测试全部通过
