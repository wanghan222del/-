# Feedback Absolute Gimbal Command Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make `rm_can_bridge` convert camera-computed relative aim offsets into lower-controller-compatible gimbal target angles using `0x402` feedback.

**Architecture:** Keep the existing relative-aim geometry in `gimbal_link`, but add an alignment layer before CAN encoding. In the new `absolute_from_feedback` mode, each new relative yaw/pitch aim sample is composed with the latest fresh `0x402` feedback so the transmitted `0x301` payload matches the lower controller's yaw/pitch semantics.

**Tech Stack:** ROS 2 Humble, C++17, `rclcpp`, existing `rm_can_bridge` q10.6 helpers, `ament_cmake_gtest`.

---

### Task 1: Add pure alignment math tests

**Files:**
- Create: `rm_can_bridge/test/test_feedback_absolute_aim.cpp`
- Create: `rm_can_bridge/include/rm_can_bridge/feedback_absolute_aim.hpp`
- Modify: `rm_can_bridge/CMakeLists.txt`

**Step 1: Write the failing test**
- Cover yaw wraparound across `±180°`.
- Cover pitch composition as `feedback + relative_offset`.

**Step 2: Run test to verify it fails**
- Run: `colcon test --packages-select rm_can_bridge --ctest-args -R test_feedback_absolute_aim`

**Step 3: Write minimal implementation**
- Add a header-only helper for `wrapDegrees180` and `composeAbsoluteAimFromFeedback`.

**Step 4: Run test to verify it passes**
- Run: `colcon test --packages-select rm_can_bridge --ctest-args -R test_feedback_absolute_aim`

### Task 2: Integrate 0x402 feedback into command generation

**Files:**
- Modify: `rm_can_bridge/src/rm_can_bridge_node.cpp`
- Modify: `rm_can_bridge/config/tracker_target_bridge.yaml`

**Step 1: Write or update failing behavior checks**
- Reuse the pure helper tests to lock command semantics before node changes.

**Step 2: Write minimal implementation**
- Add `aim_reference_mode` and `feedback_timeout_ms` parameters.
- Store latest feedback yaw/pitch and timestamp from `0x402`.
- In `absolute_from_feedback` mode, compose outgoing command angles from relative aim plus fresh feedback.
- If feedback is unavailable or stale, clear aim instead of sending a mismatched command.

**Step 3: Run targeted tests**
- Run: `colcon test --packages-select rm_can_bridge`

### Task 3: Update operator-facing defaults

**Files:**
- Modify: `rm_can_bridge/CAN_MAPPING_SUPPLEMENT.md`

**Step 1: Document command semantics**
- Note that `0x301` yaw/pitch are now lower-controller-compatible target angles when `aim_reference_mode=absolute_from_feedback`.
- Document the exact formula using `0x402` feedback.

**Step 2: Re-run package tests**
- Run: `colcon test --packages-select rm_can_bridge`
