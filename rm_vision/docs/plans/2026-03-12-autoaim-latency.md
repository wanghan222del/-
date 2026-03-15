# Auto-Aim Low-Latency Detector Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Refactor the `vision_bringup_can` auto-aim image path so the detector prioritizes the latest frame, lowers end-to-end latency, and improves average processing FPS without changing tracker or CAN bridge message interfaces.

**Architecture:** Switch the camera/detector container to `component_container_mt`, add a latest-frame handoff plus a single detector worker thread inside `ArmorDetectorNode`, cache runtime parameters outside the hot path, and expose lightweight runtime stats for latency/throughput trade-offs. Keep `/detector/armors` and `/tracker/target` interfaces unchanged.

**Tech Stack:** ROS 2 Humble, `rclcpp` components, C++14, OpenCV, `ament_cmake_gtest`, Python launch files

---

### Task 1: Add a latest-frame slot helper

**Files:**
- Create: `rm_auto_aim/armor_detector/include/armor_detector/latest_frame_slot.hpp`
- Create: `rm_auto_aim/armor_detector/test/test_latest_frame_slot.cpp`
- Modify: `rm_auto_aim/armor_detector/CMakeLists.txt`

**Step 1: Write the failing test**

```cpp
TEST(LatestFrameSlotTest, NewFrameOverwritesOlderPendingFrame)
{
  rm_auto_aim::LatestFrameSlot<int> slot;
  slot.push(1);
  slot.push(2);

  int value = 0;
  ASSERT_TRUE(slot.try_take(value));
  EXPECT_EQ(value, 2);
  EXPECT_EQ(slot.overwritten_count(), 1u);
}

TEST(LatestFrameSlotTest, StopUnblocksWaitingConsumer)
{
  rm_auto_aim::LatestFrameSlot<int> slot;
  slot.stop();

  int value = 0;
  EXPECT_FALSE(slot.wait_and_take(value));
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector --ctest-args -R test_latest_frame_slot --output-on-failure
```

Expected: FAIL because `latest_frame_slot.hpp` and the test target do not exist yet.

**Step 3: Write minimal implementation**

```cpp
template <typename T>
class LatestFrameSlot
{
public:
  void push(const T & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_value_) {
      ++overwritten_count_;
    }
    value_ = value;
    has_value_ = true;
    cv_.notify_one();
  }

  bool try_take(T & out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_value_) {
      return false;
    }
    out = value_;
    has_value_ = false;
    return true;
  }

  bool wait_and_take(T & out)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() { return stopped_ || has_value_; });
    if (!has_value_) {
      return false;
    }
    out = value_;
    has_value_ = false;
    return true;
  }

  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    cv_.notify_all();
  }
};
```

Register the new gtest in `rm_auto_aim/armor_detector/CMakeLists.txt`.

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector --ctest-args -R test_latest_frame_slot --output-on-failure
colcon test-result --verbose
```

Expected: `test_latest_frame_slot` PASS.

**Step 5: Commit**

```bash
cd /home/thom/RM_version_ws/rm_auto_aim
git add armor_detector/include/armor_detector/latest_frame_slot.hpp \
        armor_detector/test/test_latest_frame_slot.cpp \
        armor_detector/CMakeLists.txt
git commit -m "test: add latest-frame slot helper coverage"
```

### Task 2: Add detector runtime config cache

**Files:**
- Create: `rm_auto_aim/armor_detector/include/armor_detector/detector_runtime_config.hpp`
- Create: `rm_auto_aim/armor_detector/test/test_detector_runtime_config.cpp`
- Modify: `rm_auto_aim/armor_detector/CMakeLists.txt`

**Step 1: Write the failing test**

```cpp
TEST(DetectorRuntimeConfigTest, AppliesParameterUpdates)
{
  rm_auto_aim::DetectorRuntimeConfig config;
  config.binary_thres = 80;
  config.detect_color = 0;
  config.classifier_threshold = 0.8;

  config.apply(
    std::vector<rclcpp::Parameter>{
      rclcpp::Parameter("binary_thres", 96),
      rclcpp::Parameter("detect_color", 1),
      rclcpp::Parameter("classifier_threshold", 0.65)});

  EXPECT_EQ(config.binary_thres, 96);
  EXPECT_EQ(config.detect_color, 1);
  EXPECT_DOUBLE_EQ(config.classifier_threshold, 0.65);
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector --ctest-args -R test_detector_runtime_config --output-on-failure
```

Expected: FAIL because `detector_runtime_config.hpp` and the new test target do not exist yet.

**Step 3: Write minimal implementation**

```cpp
struct DetectorRuntimeConfig
{
  int binary_thres{80};
  int detect_color{0};
  double classifier_threshold{0.8};
  bool debug{false};
  double debug_image_fps{10.0};
  bool debug_publish_result_img{false};
  bool debug_publish_binary_img{false};
  bool debug_publish_number_img{false};
  int stats_log_period_ms{1000};

  void apply(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & param : params) {
      if (param.get_name() == "binary_thres") binary_thres = param.as_int();
      if (param.get_name() == "detect_color") detect_color = param.as_int();
      if (param.get_name() == "classifier_threshold") classifier_threshold = param.as_double();
    }
  }
};
```

Keep the cache focused on values read in the hot path. Do not move unrelated state into it.

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector --ctest-args -R test_detector_runtime_config --output-on-failure
colcon test-result --verbose
```

Expected: `test_detector_runtime_config` PASS.

**Step 5: Commit**

```bash
cd /home/thom/RM_version_ws/rm_auto_aim
git add armor_detector/include/armor_detector/detector_runtime_config.hpp \
        armor_detector/test/test_detector_runtime_config.cpp \
        armor_detector/CMakeLists.txt
git commit -m "test: add detector runtime config cache coverage"
```

### Task 3: Refactor `ArmorDetectorNode` to use a worker thread

**Files:**
- Modify: `rm_auto_aim/armor_detector/include/armor_detector/detector_node.hpp`
- Modify: `rm_auto_aim/armor_detector/src/detector_node.cpp`
- Modify: `rm_auto_aim/armor_detector/test/test_node_startup.cpp`

**Step 1: Write the failing test**

Add a regression test that asserts the threaded detector node declares its new runtime parameters and shuts down cleanly:

```cpp
TEST(ArmorDetectorNodeTest, DeclaresThreadedRuntimeParameters)
{
  auto node = std::make_shared<rm_auto_aim::ArmorDetectorNode>(rclcpp::NodeOptions{});
  EXPECT_TRUE(node->has_parameter("stats_log_period_ms"));
  EXPECT_TRUE(node->has_parameter("latest_frame_policy"));
}
```

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector --ctest-args -R test_node_startup --output-on-failure
```

Expected: FAIL because the new parameters are not declared yet.

**Step 3: Write minimal implementation**

Implement the refactor with these boundaries:

```cpp
void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  ++input_frame_count_;
  latest_frame_slot_.push(msg);
}

void ArmorDetectorNode::workerLoop()
{
  sensor_msgs::msg::Image::ConstSharedPtr msg;
  while (latest_frame_slot_.wait_and_take(msg)) {
    auto armors = detectArmors(msg);
    publishDetectionResult(msg, armors);
    ++processed_frame_count_;
  }
}
```

Implementation checklist:

- Add `LatestFrameSlot<sensor_msgs::msg::Image::ConstSharedPtr>` member.
- Add `std::thread worker_thread_`, stop flag, and clean shutdown in the destructor.
- Move all direct `detector_` access to the worker thread.
- Replace hot-path `get_parameter()` calls with cached config reads.
- Add throttled runtime stats logs using counters such as `input_frame_count_`, `processed_frame_count_`, and `overwritten_frame_count_`.
- Keep `/detector/armors` publishing semantics unchanged.

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector --ctest-args -R "test_node_startup|test_latest_frame_slot|test_detector_runtime_config" --output-on-failure
colcon test-result --verbose
```

Expected: all three armor detector tests PASS.

**Step 5: Commit**

```bash
cd /home/thom/RM_version_ws/rm_auto_aim
git add armor_detector/include/armor_detector/detector_node.hpp \
        armor_detector/src/detector_node.cpp \
        armor_detector/test/test_node_startup.cpp
git commit -m "feat: process detector images on latest-frame worker"
```

### Task 4: Switch `vision_bringup_can` to low-latency defaults

**Files:**
- Create: `rm_vision/rm_vision_bringup/test/test_vision_bringup_can_launch.py`
- Modify: `rm_vision/rm_vision_bringup/CMakeLists.txt`
- Modify: `rm_vision/rm_vision_bringup/package.xml`
- Modify: `rm_vision/rm_vision_bringup/launch/vision_bringup_can.launch.py`
- Modify: `rm_vision/rm_vision_bringup/config/node_params.yaml`

**Step 1: Write the failing test**

Create a Python launch smoke test that loads the launch description and asserts the detector container uses the multithreaded executable:

```python
from rm_vision_bringup.launch.vision_bringup_can import generate_launch_description


def test_vision_bringup_can_uses_multithreaded_container():
    launch_description = generate_launch_description()
    containers = [entity for entity in launch_description.entities if getattr(entity, "name", "") == "camera_detector_container"]
    assert containers, "camera_detector_container not found"
    assert containers[0].executable == "component_container_mt"
```

**Step 2: Run test to verify it fails**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select rm_vision_bringup --ctest-args -R test_vision_bringup_can_launch --output-on-failure
```

Expected: FAIL because the test target does not exist yet and the launch file still uses `component_container`.

**Step 3: Write minimal implementation**

Make the following changes:

```python
return ComposableNodeContainer(
    name='camera_detector_container',
    package='rclcpp_components',
    executable='component_container_mt',
    ...
)
```

And in `node_params.yaml`, switch the defaults toward low-latency operation:

```yaml
/armor_detector:
  ros__parameters:
    debug: false
    debug_image_fps: 10.0
    debug_publish_result_img: false
    debug_publish_binary_img: false
    debug_publish_number_img: false
```

Update CMake/package metadata as needed to run the Python launch test.

**Step 4: Run test to verify it passes**

Run:

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select rm_vision_bringup --ctest-args -R test_vision_bringup_can_launch --output-on-failure
colcon test-result --verbose
```

Expected: the new launch smoke test PASS.

**Step 5: Commit**

```bash
cd /home/thom/RM_version_ws/rm_vision
git add rm_vision_bringup/test/test_vision_bringup_can_launch.py \
        rm_vision_bringup/CMakeLists.txt \
        rm_vision_bringup/package.xml \
        rm_vision_bringup/launch/vision_bringup_can.launch.py \
        rm_vision_bringup/config/node_params.yaml
git commit -m "feat: enable low-latency can bringup defaults"
```

### Task 5: Verify end-to-end latency and throughput

**Files:**
- Modify: `rm_vision/docs/plans/2026-03-12-autoaim-latency-design.md`
- Modify: `rm_vision/docs/plans/2026-03-12-autoaim-latency.md`

**Step 1: Run the targeted automated tests**

```bash
cd /home/thom/RM_version_ws
colcon test --packages-select armor_detector rm_vision_bringup --event-handlers console_direct+
colcon test-result --verbose
```

Expected: PASS for the new detector and bringup tests.

**Step 2: Run runtime smoke validation**

```bash
cd /home/thom/RM_version_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup_can.launch.py
```

In another shell:

```bash
ros2 topic hz /image_raw
ros2 topic hz /detector/armors
ros2 topic hz /tracker/target
```

Expected: `/detector/armors` and `/tracker/target` update rates improve or stay stable while perceived lag decreases.

**Step 3: Record before/after measurements**

Capture at least:

- input FPS
- detector FPS
- tracker FPS
- average processing time per frame
- overwritten-frame count over a fixed 30-second window

**Step 4: Update the docs with measured numbers**

Add a short “Measured Results” section to both plan documents so the final merge request shows the real trade-offs.

**Step 5: Commit**

```bash
cd /home/thom/RM_version_ws/rm_vision
git add docs/plans/2026-03-12-autoaim-latency-design.md \
        docs/plans/2026-03-12-autoaim-latency.md
git commit -m "docs: record autoaim latency verification results"
```

## 自动化验证结果

- 2026-03-12 在当前工作空间执行：`colcon test --packages-select armor_detector rm_vision_bringup --event-handlers console_direct+ && colcon test-result --verbose`
- 结果：`64 tests, 0 errors, 0 failures, 15 skipped`
- 关键覆盖：
  - `armor_detector`：`test_node_startup`、`test_latest_frame_slot`、`test_detector_runtime_config`、`clang_format`、`cppcheck`、`xmllint`
  - `rm_vision_bringup`：`test_vision_bringup_can_launch`
- 额外确认：`vision_bringup_can.launch.py` 的相机/检测容器已切换为 `component_container_mt`，smoke test 通过

## 硬件验证待办

当前会话没有连接真实相机/云台/CAN 硬件，因此以下指标还需要你在目标设备上补测：

- `/image_raw` 输入 FPS
- `/detector/armors` 输出 FPS
- `/tracker/target` 输出 FPS
- 端到端检测延迟
- 最新帧覆盖计数在 30 秒窗口内的变化

建议使用：

```bash
ros2 topic hz /image_raw
ros2 topic hz /detector/armors
ros2 topic hz /tracker/target
```

配合 Foxglove 观察画面时效性变化，但最终以节点统计与 topic 频率为准。
