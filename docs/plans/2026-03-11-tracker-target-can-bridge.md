# Tracker Target CAN Bridge Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a CAN-output bringup path that keeps the existing auto-aim pipeline intact, reads `/tracker/target`, computes relative yaw/pitch with muzzle-offset compensation, sends the result through the current USB-CAN serial device, and receives `0x402` q10.6 gimbal feedback in the same node.

**Architecture:** Extend `rm_can_bridge` with a new `tracker_target` input mode instead of replacing the package. Keep `rm_vision_bringup` as the main system entrypoint by adding a CAN-specific launch that removes `rm_serial_driver`, overrides `armor_tracker.target_frame` to `gimbal_link`, and starts a single `rm_can_bridge` instance responsible for both `0x301` send and `0x402` receive.

**Tech Stack:** ROS 2 Humble, C++17, `rclcpp`, `auto_aim_interfaces`, `geometry_msgs`, `std_msgs`, launch/launch_ros, existing USB-CAN serial framing in `rm_can_bridge`, `ament_cmake_gtest` for geometry/q10.6 tests.

---

### Task 1: Add testable geometry and q10.6 helpers

**Files:**
- Create: `rm_can_bridge/include/rm_can_bridge/target_aim_math.hpp`
- Create: `rm_can_bridge/include/rm_can_bridge/q10_6_angle.hpp`
- Create: `rm_can_bridge/test/test_target_aim_math.cpp`
- Create: `rm_can_bridge/test/test_q10_6_angle.cpp`
- Modify: `rm_can_bridge/CMakeLists.txt`
- Modify: `rm_can_bridge/package.xml`

**Step 1: Write the failing tests**

Add geometry tests and q10.6 tests.

```cpp
TEST(TargetAimMath, ComputesZeroAnglesWhenTargetIsStraightAhead) { ... }
TEST(TargetAimMath, AppliesMuzzleOffsetBeforeAngleCalculation) { ... }
TEST(Q10_6Angle, Converts15DegTo960) {
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(15.0), 960);
}
TEST(Q10_6Angle, Converts960To15Deg) {
  EXPECT_NEAR(rm_can_bridge::q10_6RawToDeg(960), 15.0, 1e-6);
}
```

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select rm_can_bridge --cmake-clean-cache && colcon test --packages-select rm_can_bridge --ctest-args -R 'test_target_aim_math|test_q10_6_angle' --output-on-failure`

Expected: FAIL before both helpers exist.

**Step 3: Write minimal implementation**

Implement:

- `computeRelativeAim(target, muzzle)` returning yaw/pitch in radians
- `degToQ10_6Raw(double)` matching the STM32 `q_10_6_t` semantics (`round(value * 64)`)
- `q10_6RawToDeg(int16_t)` returning float/double deg

**Step 4: Run test to verify it passes**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select rm_can_bridge && colcon test --packages-select rm_can_bridge --ctest-args -R 'test_target_aim_math|test_q10_6_angle' --output-on-failure && colcon test-result --all`

Expected: PASS for both test targets.

**Step 5: Commit**

```bash
git add rm_can_bridge/include/rm_can_bridge/target_aim_math.hpp \
        rm_can_bridge/include/rm_can_bridge/q10_6_angle.hpp \
        rm_can_bridge/test/test_target_aim_math.cpp \
        rm_can_bridge/test/test_q10_6_angle.cpp \
        rm_can_bridge/CMakeLists.txt \
        rm_can_bridge/package.xml
git commit -m "test: add geometry and q10.6 coverage"
```

### Task 2: Extend `rm_can_bridge` with `tracker_target` send mode

**Files:**
- Modify: `rm_can_bridge/src/rm_can_bridge_node.cpp`
- Modify: `rm_can_bridge/launch/rm_can_bridge.launch.py`
- Create: `rm_can_bridge/config/tracker_target_bridge.yaml`

**Step 1: Write the failing test**

Add a geometry/encoding assertion for 15°/15° after muzzle compensation.

```cpp
TEST(TargetAimMath, Encodes15DegToQ10_6) {
  const auto angles = rm_can_bridge::computeRelativeAim({1.0, 0.26794919, 0.27740142}, {0.0, 0.0, 0.0});
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(angles.yaw_rad * 180.0 / M_PI), 960);
  EXPECT_EQ(rm_can_bridge::degToQ10_6Raw(angles.pitch_rad * 180.0 / M_PI), 960);
}
```

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/humble/setup.bash && colcon test --packages-select rm_can_bridge --ctest-args -R test_target_aim_math --output-on-failure`

Expected: FAIL until the helper/API is wired consistently.

**Step 3: Write minimal implementation**

Extend `rm_can_bridge` to support:

- `input_mode` (`detector_armors` / `tracker_target`)
- `target_topic` (`/tracker/target`)
- `target_timeout_ms`
- `zero_when_lost`
- `muzzle_offset_xyz_in_gimbal`
- `publish_feedback`
- `feedback_can_id`

When `input_mode == tracker_target`:

- subscribe to `/tracker/target`
- require `msg->tracking == true`
- treat `msg->position` as `gimbal_link` coordinates
- subtract `muzzle_offset_xyz_in_gimbal`
- compute relative yaw/pitch
- encode to `0x301` using **deg q10.6**
- keep the existing send timers and `0x301` frame layout

Preserve the current `/detector/armors` mode as the default backward-compatible behavior.

Create `rm_can_bridge/config/tracker_target_bridge.yaml` with the tracker-target parameters.

**Step 4: Run test to verify it passes**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select rm_can_bridge && colcon test --packages-select rm_can_bridge --ctest-args -R test_target_aim_math --output-on-failure && colcon test-result --all`

Expected: PASS for geometry/encoding tests.

**Step 5: Commit**

```bash
git add rm_can_bridge/src/rm_can_bridge_node.cpp \
        rm_can_bridge/launch/rm_can_bridge.launch.py \
        rm_can_bridge/config/tracker_target_bridge.yaml \
        rm_can_bridge/include/rm_can_bridge/target_aim_math.hpp \
        rm_can_bridge/include/rm_can_bridge/q10_6_angle.hpp \
        rm_can_bridge/test/test_target_aim_math.cpp
git commit -m "feat: add tracker target CAN send mode"
```

### Task 3: Add `0x402` q10.6 feedback receive path

**Files:**
- Modify: `rm_can_bridge/src/rm_can_bridge_node.cpp`
- Create: `rm_can_bridge/test/test_q10_6_angle.cpp`

**Step 1: Write the failing test**

Add q10.6 decode tests covering the 0x402 field semantics.

```cpp
TEST(Q10_6Angle, DecodesNegativePitch) {
  EXPECT_NEAR(rm_can_bridge::q10_6RawToDeg(static_cast<int16_t>(-960)), -15.0, 1e-6);
}
```

**Step 2: Run test to verify it fails**

Run: `source /opt/ros/humble/setup.bash && colcon test --packages-select rm_can_bridge --ctest-args -R test_q10_6_angle --output-on-failure`

Expected: FAIL before the helper covers this case.

**Step 3: Write minimal implementation**

In `rm_can_bridge`:

- add a non-blocking USB-CAN receive loop/timer
- parse full 30-byte USB-CAN frames
- filter `CAN ID == 0x402`
- decode four q10.6 fields from payload bytes `[0..7]`
- immediately convert to **deg floating-point values**
- publish:
  - `/gimbal/feedback/rpy_deg` as `geometry_msgs/msg/Vector3Stamped` (`x=roll`, `y=pitch`, `z=yaw`)
  - `/gimbal/feedback/yaw_motor_pos_deg` as `std_msgs/msg/Float64`

Do not expose the feedback as “plain int angles” inside ROS logic.

**Step 4: Run test to verify it passes**

Run: `source /opt/ros/humble/setup.bash && colcon build --packages-select rm_can_bridge && colcon test --packages-select rm_can_bridge --ctest-args -R 'test_target_aim_math|test_q10_6_angle' --output-on-failure && colcon test-result --all`

Expected: PASS for both helper test targets, with build succeeding after receive-path additions.

**Step 5: Commit**

```bash
git add rm_can_bridge/src/rm_can_bridge_node.cpp \
        rm_can_bridge/include/rm_can_bridge/q10_6_angle.hpp \
        rm_can_bridge/test/test_q10_6_angle.cpp
git commit -m "feat: add 0x402 q10.6 feedback receive path"
```

### Task 4: Add CAN-specific vision bringup

**Files:**
- Modify: `rm_vision/rm_vision_bringup/launch/common.py`
- Create: `rm_vision/rm_vision_bringup/launch/vision_bringup_can.launch.py`

**Step 1: Write the failing test**

Run before implementation:

```bash
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash
timeout 8 ros2 launch rm_vision_bringup vision_bringup_can.launch.py
```

Expected: FAIL because `vision_bringup_can.launch.py` does not exist yet.

**Step 2: Run test to verify it fails**

Run the exact command above.

Expected: launch file not found.

**Step 3: Write minimal implementation**

Implement a new bringup launch that:

- reuses the existing robot description helper from `common.py`
- creates the same camera + detector container used by bringup
- creates a tracker node with parameter override `{"target_frame": "gimbal_link"}`
- removes `rm_serial_driver`
- starts `rm_can_bridge` with `tracker_target_bridge.yaml`

Refactor `common.py` just enough to expose a reusable tracker-node factory instead of hard-coding a single global `tracker_node` instance.

**Step 4: Run test to verify it passes**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash
timeout 8 ros2 launch rm_vision_bringup vision_bringup_can.launch.py
```

Expected: launch starts the camera container, tracker node, robot state publisher, and CAN bridge without trying to start `rm_serial_driver`.

**Step 5: Commit**

```bash
git add rm_vision/rm_vision_bringup/launch/common.py \
        rm_vision/rm_vision_bringup/launch/vision_bringup_can.launch.py
git commit -m "feat: add CAN tracker bringup launch"
```

### Task 5: Verify end-to-end behavior on hardware and update docs

**Files:**
- Modify: `rm_can_bridge/CAN_MAPPING_SUPPLEMENT.md`
- Modify: `rm_vision/README.md`

**Step 1: Write the failing test**

Use a manual regression checklist because this step depends on real camera + CAN hardware.

Checklist to fail before documentation update:

- No documented command exists for `vision_bringup_can.launch.py`
- No documented parameter explains `muzzle_offset_xyz_in_gimbal`
- No documented explanation exists for `0x402` -> ROS feedback topics

**Step 2: Run test to verify it fails**

Run the checklist manually against the current docs.

Expected: all three items are missing.

**Step 3: Write minimal implementation**

Document:

- the new launch command
- the new config file path
- the meaning and tuning direction of `muzzle_offset_xyz_in_gimbal`
- the `0x402` field mapping and deg output semantics
- the feedback topics
- a manual CAN validation command using `/tracker/target`

Suggested validation commands to include:

```bash
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash
ros2 launch rm_vision_bringup vision_bringup_can.launch.py
```

```bash
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash
ros2 param get /armor_tracker target_frame
```

```bash
source /opt/ros/humble/setup.bash
source /home/thom/RM_version_ws/install/setup.bash
ros2 topic pub -r 50 /tracker/target auto_aim_interfaces/msg/Target "{header: {frame_id: 'gimbal_link'}, tracking: true, id: 'manual', armors_num: 2, position: {x: 1.0, y: 0.26794919, z: 0.27740142}, velocity: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, v_yaw: 0.0, radius_1: 0.0, radius_2: 0.0, dz: 0.0}"
```

Expected CAN analyzer result for `0x301` with zero muzzle offset and `deg` mode: payload yaw/pitch near `03 c0 03 c0`.

**Step 4: Run test to verify it passes**

Run the checklist again after doc updates and verify each command works on hardware.

Expected: documentation matches the actual runtime behavior.

**Step 5: Commit**

```bash
git add rm_can_bridge/CAN_MAPPING_SUPPLEMENT.md \
        rm_vision/README.md
git commit -m "docs: describe tracker target CAN bringup"
```
