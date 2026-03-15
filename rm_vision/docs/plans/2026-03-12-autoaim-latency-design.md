# 自瞄低延迟高帧率检测设计方案

## 背景

当前自瞄系统通过 `ros2 launch rm_vision_bringup vision_bringup_can.launch.py` 启动。该链路中：

- 相机驱动节点和 `armor_detector` 组件被放在同一个组件容器中。
- `vision_bringup_can.launch.py` 当前使用的是 `component_container`，不是 `component_container_mt`。
- 海康驱动内部已经有独立采集线程，但 `armor_detector` 的 `imageCallback()` 仍然以单回调串行方式完成整帧处理。
- 默认配置中 `armor_detector` 还开启了调试图像输出，调试发布会进一步占用 CPU、内存带宽和 ROS 发布带宽。

这使系统容易出现“平均 FPS 不高、旧帧排队、端到端延迟抬升”的问题。对自瞄而言，旧帧价值迅速下降，因此需要优先保证“处理最新帧”的能力，同时尽量提高平均处理吞吐。

## 目标

1. 优先处理最新帧，显著降低检测链路延迟。
2. 在低延迟优先的前提下，提高 `armor_detector` 的平均处理 FPS。
3. 保持 `/detector/armors`、`/tracker/target` 话题接口不变，不改 tracker 与 CAN bridge 的消息协议。
4. 控制改动范围，优先修改 `rm_vision_bringup` 与 `armor_detector`，不引入整套复杂线程池。
5. 为后续调优提供可观测指标，包括输入 FPS、处理 FPS、覆盖丢帧数和端到端延迟。

## 非目标

- 不重写 `armor_tracker` 的 TF/filter 架构。
- 不引入新的自定义消息类型来承载性能统计。
- 不实现多级无锁流水线或线程池并行分类/PnP。
- 不修改海康 SDK 的底层采集机制。

## 现状与瓶颈

### 1. 启动编排层

`vision_bringup_can.launch.py` 使用单线程组件容器承载相机和检测器。即使海康驱动内部有采集线程，ROS 组件层仍然可能因为检测回调较重而形成回调竞争和消息堆积。

### 2. 检测节点热路径过长

`ArmorDetectorNode::imageCallback()` 当前直接完成以下步骤：

1. 图像转 `cv::Mat`
2. 图像预处理
3. 灯条查找与配对
4. 数字分类
5. PnP 解算
6. marker 与 debug 图像发布

这意味着一帧处理期间，节点无法快速接纳新帧。

### 3. 参数读取与调试发布影响热路径

当前检测热路径中每帧读取若干参数；同时调试图像默认启用结果图发布。对于低算力平台，这些额外成本会侵蚀本就有限的处理预算。

## 方案比较

### 方案 A：只把容器改成多线程

做法：

- `vision_bringup_can.launch.py` 改用 `component_container_mt`
- 保持 `armor_detector` 内部结构不变
- 适当降低 debug 发布频率

优点：

- 改动最小
- 风险最低

缺点：

- 检测节点仍是单回调串行处理
- 旧帧排队问题无法根治
- 收益通常有限

### 方案 B：最新帧优先的混合并发方案

做法：

- 启动层改成多线程组件容器
- `imageCallback()` 只负责收帧并覆盖“最新帧槽位”
- 新增单独检测工作线程，始终处理当前最新帧
- 调试输出降频或异步处理

优点：

- 最符合自瞄“旧帧迅速贬值”的业务特点
- 能同时降低延迟并提升平均吞吐
- 结构清晰，易于验证和维护

缺点：

- 需要重构 `armor_detector` 内部结构
- 需要谨慎处理线程安全和节点析构

### 方案 C：完整多级流水线并行

做法：

- 预处理、候选提取、分类、PnP 各自分线程
- 通过队列或环形缓冲传递中间结果

优点：

- 理论吞吐最高

缺点：

- 复杂度显著提高
- 调试与时间戳一致性变差
- 对当前仓库和低算力目标平台而言性价比不高

## 选型结论

采用 **方案 B：最新帧优先的混合并发方案**。

核心原则：

- **低延迟优先**：宁可覆盖旧待处理帧，也不让旧帧在队列里排长队。
- **吞吐尽量保住**：通过解耦收帧与检测，让 CPU 更连续地做有效工作。
- **改动集中**：只在 `rm_vision_bringup` 与 `armor_detector` 两处做关键变更。

## 详细设计

### 一、启动层改动

将 `vision_bringup_can.launch.py` 中相机+检测容器从 `component_container` 改为 `component_container_mt`。

目的：

- 让组件层具备并发调度能力。
- 避免相机驱动发布与检测节点回调完全串在同一执行上下文里。

该改动本身不是最终收益来源，但为后续的 detector 内部解耦提供更合理的运行环境。

### 二、`armor_detector` 改成“轻回调 + 单工作线程”

新的 `ArmorDetectorNode` 内部结构如下：

1. **ROS 收帧回调**
   - `imageCallback()` 不再直接做整帧检测。
   - 只负责保存最新图像消息、记录到达计数和时间戳、唤醒工作线程。
   - 回调必须尽量短，目标是“快速接纳下一帧”。

2. **最新帧槽位（Latest Frame Slot）**
   - 节点内部只保留一个待处理槽位。
   - 如果工作线程忙，新帧到来时直接覆盖旧的待处理帧。
   - 同时累计“覆盖丢帧计数”，供后续分析。

3. **检测工作线程**
   - 节点启动后创建一个工作线程。
   - 工作线程等待最新帧槽位就绪，然后处理整帧：预处理、找灯条、配对、分类、PnP、结果发布。
   - 任意时刻只允许这一个线程访问 `detector_` 与调试图像缓存，避免数据竞争。

### 三、参数缓存

当前实现中，检测热路径每帧使用 `get_parameter()` 获取阈值与分类器参数。新的设计将引入参数缓存：

- 节点启动时读取一次运行参数。
- 通过参数回调维护缓存结构体。
- 工作线程只读取缓存，不在热路径中走参数查询接口。

缓存内容至少包括：

- `binary_thres`
- `detect_color`
- `classifier_threshold`
- debug 图像开关
- debug 发布频率
- 统计日志周期

### 四、调试输出策略

调试信息分级处理：

1. `/detector/armors` 与 marker 发布为主功能输出，优先级最高。
2. 结果图、binary 图、number 图属于可丢弃调试输出，默认降低频率。
3. 正式自瞄运行时默认关闭非必要 debug 图像；调参时再打开。

默认配置建议：

- `debug` 默认改为 `false`
- `debug_publish_result_img` 默认改为 `false`
- 如果需要保留调试，可将 `debug_image_fps` 降到 5~10 FPS

### 五、线程安全约束

为避免复杂锁竞争，明确以下规则：

1. **只有收帧回调可以写最新帧槽位**。
2. **只有工作线程可以读出并消费最新帧槽位**。
3. **只有工作线程可以访问 `detector_` 内部可变状态**。
4. `pnp_solver_` 在 `camera_info` 初始化完成后只读使用。
5. 统计计数器可使用原子变量，避免高频日志锁竞争。

### 六、错误处理与退出流程

错误处理策略：

- `cv_bridge` 转换失败：记录节流错误日志，跳过该帧。
- `PnP` 失败：降低为节流警告或计数，不对每帧刷日志。
- OpenCV/分类器异常：在工作线程内部捕获，记录计数后继续运行。
- `camera_info` 未就绪：允许检测继续进行，但在姿态输出环节安全跳过。

退出流程：

1. 设置 `stop` 标志。
2. 唤醒工作线程。
3. `join()` 工作线程。
4. 再析构发布器、检测器和相关资源。

这样可以避免节点析构后后台线程仍访问 ROS 对象。

## 数据流

新的数据流如下：

1. 海康驱动采集线程从相机抓帧。
2. 相机组件发布 `/image_raw`。
3. `ArmorDetectorNode::imageCallback()` 仅保存最新帧并立即返回。
4. 检测工作线程拿到最新帧后执行完整检测。
5. 发布 `/detector/armors` 给 `armor_tracker`。
6. `armor_tracker` 与 `rm_can_bridge` 维持原接口继续工作。

## QoS 与丢帧策略

设计原则：**最新帧优先，而不是完整帧保真优先**。

- 输入图像继续使用 `SensorDataQoS`。
- 节点内部不使用长队列；只保留“正在处理的一帧”和“最新待处理帧”。
- 当工作线程落后于相机输入时，优先覆盖旧待处理帧。
- 该策略会增加“逻辑丢帧”，但换来更低的结果时效性延迟。

## 观测指标与验收标准

### 运行指标

至少记录以下统计项：

- 输入帧数与输入 FPS
- 实际处理帧数与处理 FPS
- 覆盖丢帧次数
- 最近一次处理耗时
- 从图像时间戳到结果发布的端到端延迟

### 验收标准

与改造前相比，满足以下目标则认为设计达标：

1. `vision_bringup_can` 下 `armor_detector` 平均处理 FPS 提升。
2. 检测结果时间戳与当前场景更接近，目标滞后感明显减轻。
3. `tracker` 输出更新更稳定，不再明显消费很旧的检测结果。
4. 调试输出关闭后，CPU 峰值与波动下降。

## 测试策略

### 单元测试

- 为“最新帧槽位”写独立 gtest，验证覆盖行为与停止逻辑。
- 为参数缓存写独立 gtest，验证更新逻辑不依赖热路径查询。
- 为 `ArmorDetectorNode` 启动/析构写回归测试，确保后台线程可正常退出。

### 集成验证

- `colcon test --packages-select armor_detector`
- `colcon test --packages-select rm_vision_bringup`
- 运行 `vision_bringup_can.launch.py` 后，用 `ros2 topic hz` 对 `/image_raw`、`/detector/armors`、`/tracker/target` 做前后对比。

## 风险与缓解

### 风险 1：覆盖旧帧导致统计上“掉帧变多”

缓解：

- 明确把“时效性”作为首要优化目标。
- 使用覆盖计数器量化该权衡，而不是盲目追求零丢帧。

### 风险 2：线程退出不干净导致节点析构卡住

缓解：

- 使用明确的停止标志、条件变量和 `join()` 顺序。
- 用启动/析构测试覆盖该场景。

### 风险 3：调试功能影响主链路

缓解：

- 将调试输出默认关闭。
- 将 debug 发布与主检测结果拆分优先级。

## 总结

本方案的核心不是“让更多线程同时抢着处理所有帧”，而是让系统始终把有限算力优先花在**最新的有效图像**上。通过“多线程组件容器 + 最新帧槽位 + 单检测工作线程 + 参数缓存 + 调试降频”这组组合，可以在不大幅扩大系统复杂度的前提下，较稳妥地实现**更低延迟**和**更高平均处理 FPS**。

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
