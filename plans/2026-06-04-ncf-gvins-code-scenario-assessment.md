# NcF-GVINS 代码级场景能力评估报告

## 0. 报告边界

本文是纯代码、配置、launch、脚本和数据链路分析，不包含编译、rosbag 回放、evo 评估或批处理运行。文中结论按以下证据等级标注：

- `代码事实`：可由当前仓库文件直接确认。
- `基于代码的合理推断`：由数据链路、状态机、配置开关和因子设计推导得到，但仍需要后续数据验证。
- `需实验验证`：当前代码可支持设计验证方案，但本文不提供运行结果。

重点场景包括：室外、室内、室内外过渡、室外天空大面积遮蔽、视觉退化、IMU 固定不稳或低成本 IMU、部分传感器起步失效后恢复。

## 1. 系统数据链路与逻辑架构

### 1.1 ROS 输入、主节点与输出

`代码事实`：主融合节点在 `ic_gvins/ROS/fusion_ros.cc` 中读取私有参数 `imu_topic`、`gnss_topic`、`image_topic`、`configfile`，默认话题分别为 `/imu0`、`/gnss0`、`/cam0`。订阅队列为 IMU 200、GNSS 50、图像 20，可选航向话题在 `nc_extension.use_magnetic_heading` 开启时订阅。

`代码事实`：主节点发布在线轨迹、GNSS 可视化、健康面板，并在 `nc_extension.enabled` 与 `enable_async_relocator` 同时开启时发布 `RecoveryFrame` 和 latched `RecoveryEvent`。关键证据：

- `fusion_ros.cc:152-158`：输入话题和配置文件参数。
- `fusion_ros.cc:231-243`：`recovery_frame`、`recovery_event`、`gnss_measurements`、`sensor_health_panel` 发布器。
- `fusion_ros.cc:250-261`：IMU、GNSS、图像和可选航向订阅。
- `fusion_ros.cc:439-498`：RecoveryFrame/RecoveryEvent 消息填充与发布。

`基于代码的合理推断`：NcF-GVINS 的在线输出和全局校正输出是解耦的。主节点维护连续 `odom`，异步重定位节点发布 `global_path`、`map_to_odom` 和 TF。这个设计适合避免 GNSS 恢复或全局校正直接冲击实时里程计，但也意味着在线估计器本身不一定被重写到全局 `map` 坐标。

### 1.2 GNSS 数据进入方式

`代码事实`：GNSS 回调从 `NavSatFix.position_covariance` 中取 `[4] -> N`、`[0] -> E`、`[8] -> D` 的标准差，要求标准差有限、大于 0 且低于 `gnssthreshold`。水平有效性成为 `quality_valid`，垂直有效性独立保存。有限 outage 注入由 `isusegnssoutage`、`gnssoutagetime`、`gnssoutageendtime` 控制。

关键证据：

- `fusion_ros.cc:327-340`：GNSS N/E/D 标准差和水平/垂直有效性。
- `fusion_ros.cc:344-346`：可恢复 outage 时间段注入。
- `fusion_ros.cc:348-355`：NcF 开启时退化 GNSS 仍进入核心作为健康证据；NcF 关闭时保留原始 IC-GVINS 的整体准入逻辑。

`基于代码的合理推断`：该设计比原始整体 GNSS 拒绝更细，但强依赖 ROS bag 中 GNSS covariance 的可信度。若 covariance 全 0、过大、未按 N/E/D 约定填充或与实际质量不一致，健康门控将偏离真实卫星质量。

### 1.3 核心融合线程

`代码事实`：`GVINS` 核心包含融合、跟踪、优化三条线程。IMU 负责传播与状态窗口推进；视觉跟踪为关键帧和重投影因子提供约束；GNSS 在时间节点插入后作为因子参与优化；NcF 扩展在 GNSS 准入前后维护健康状态、恢复段和异步重定位帧。

关键证据：

- `ic_gvins.cc:423`：IMU 数据入口。
- `ic_gvins.cc:503`：GNSS 数据入口。
- `ic_gvins.cc:913`：GNSS 在线融合准备与健康判断。
- `ic_gvins.cc:1262`：融合线程中检查 GNSS timeout。
- `ic_gvins.cc:1283`：初始化阶段尝试 local bootstrap。
- `ic_gvins.cc:1444`：优化后向异步重定位节点发出恢复帧。
- `ic_gvins.cc:2266-2270`：优化中加入 GNSS 与可选航向因子。
- `ic_gvins.cc:3169`、`ic_gvins.cc:3222`：GNSS 与航向因子加入函数。

### 1.4 健康状态机与恢复

`代码事实`：`SensorHealthManager` 使用 `ACTIVE`、`DEGRADED`、`RECOVERING`、`UNAVAILABLE` 表达多模态健康状态。GNSS 水平和垂直轴独立更新；视觉、航向、IMU 也复用同一状态机和协方差缩放机制。

关键证据：

- `sensor_health_manager.h:54-66`：水平/垂直 GNSS 独立更新。
- `sensor_health_manager.h:83-115`：视觉、航向、IMU 健康更新。
- `sensor_health_manager.h:182-218`：退化、恢复确认和协方差缩放的通用状态迁移。
- `ic_gvins.cc:836`：打开恢复段。
- `ic_gvins.cc:857`：估计恢复偏差。
- `ic_gvins.cc:1002-1058`：GNSS innovation 异常触发恢复段，恢复确认后产生事件。

`基于代码的合理推断`：NcF-GVINS 的核心思想不是让某个传感器失效后立即丢弃，而是尽量保留“可用分量”和“健康证据”，通过降权、恢复确认和异步 map 校正维护系统连续性。

### 1.5 异步重定位节点

`代码事实`：`ic_gvins/ROS/degraded_reloc_node.cc` 将在线相对运动约束和 raw GNSS 锚点约束放入 4DoF 图优化，发布 `global_path`、`map_to_odom`，并写出 `global_path.csv`、`global_path_unix.csv`。

关键证据：

- `degraded_reloc_node.cc:112-147`：相对 4DoF 因子与 raw GNSS 锚点因子。
- `degraded_reloc_node.cc:197-209`：相对约束、最大节点数、恢复锚点数参数。
- `degraded_reloc_node.cc:217-220`：`global_path` 与 `map_to_odom` 发布器。
- `degraded_reloc_node.cc:377-430`：恢复段图优化构建。
- `degraded_reloc_node.cc:493-498`：发布全局路径和 map-to-odom。
- `reloc_segment_lifecycle.h:43-73`、`reloc_segment_lifecycle.h:195-211`：恢复段从采集、恢复确认到可优化队列的生命周期。

`基于代码的合理推断`：异步重定位是系统面对 GNSS 恢复、室内外过渡和遮蔽后重获 GNSS 的主要补偿机制。其优势是隔离在线里程计，缺陷是全局修正存在异步滞后，并依赖恢复后足够 raw GNSS 锚点。

## 2. 场景能力矩阵

### 2.1 室外常规场景

**现状**

`代码事实`：系统继承 IC-GVINS 的 INS-centric GNSS/视觉/惯性紧耦合框架，默认配置面向有 GNSS、IMU、单目图像的数据。KAIST 和 i2Nav launch 分别配置了对应 IMU/GNSS/Image 话题，`config/gvins-kaist-urban38.yaml`、`gvins-kaist-urban22.yaml`、`gvins-i2nav-building00.yaml` 默认开启 NcF 与异步重定位。

**优点**

`代码事实`：GNSS 水平/垂直可独立进入因子。`gnss_factor.h:59-64` 对水平和垂直分别设置信息矩阵；`recovery_gnss_factor.h:43-48` 对恢复 GNSS 也采用同样分轴信息。

`基于代码的合理推断`：常规室外场景中，GNSS 可提供全局约束，视觉补充局部约束，IMU 提供高频传播。NcF 的水平/垂直分轴机制可降低坏高度直接屏蔽水平定位的概率。

**缺陷**

`代码事实`：GNSS 有效性主要由 `position_covariance` 与固定阈值判定；视觉为单目角点前端；IMU 要求坐标系、频率、噪声模型和杆臂配置正确。

`基于代码的合理推断`：常规室外能力强弱高度依赖标定、话题、时间戳、covariance 和 IMU 模型。若数据质量元信息缺失，健康管理可能把真实有效 GNSS 拒绝，或把伪健康 GNSS 当作可用输入。

**分层评价**

- 系统级：主干完整，ROS 链路清楚，在线 `odom` 与异步 `map` 校正分离。
- 方法级：GNSS 分轴、恢复段和视觉/IMU 降权提升了容错框架完整性。
- 思路级：从“传感器整体可用/不可用”转向“分量与模态健康管理”，方向合理。

**进步空间**

- 引入 GNSS 原始质量字段或驱动层质量分类，如 fix type、卫星数、DOP、RTK 状态、NLOS 评分。
- 增加配置自检：话题、covariance、坐标系、杆臂、频率与时间同步一致性。
- 输出更结构化的健康日志，便于离线分析健康状态切换原因。

**后续验证建议**

`需实验验证`：用 KAIST urban38/urban22 和 i2Nav 室外序列，比较 `00_ic_baseline`、`01_nc_transparent`、`08_nc_full` 的 APE/RPE、GNSS outlier 数、健康状态切换次数和运行失败率。

### 2.2 室内场景

**现状**

`代码事实`：系统支持无 GNSS 静止本地启动。`local_initializer.h:5-6` 明确该初始化器用于 GNSS 缺席时的保守 local odom；`local_initializer.cc:15-50` 使用一段静止 IMU 估计初始姿态、陀螺零偏、零位置、零速度；`ic_gvins.cc:1749-1779` 将 local bootstrap 激活为本地坐标模式。

**优点**

`代码事实`：室内起步时，只要静止 IMU 窗口满足要求，系统可以不等待 GNSS 完成局部初始化。可选 calibrated heading 可给 local yaw，否则 yaw 是局部 gauge。

`基于代码的合理推断`：对“室内先启动，之后再走到室外”的任务，local bootstrap 使系统不必在起点等待 GNSS，工程实用性明显优于纯 GNSS 起步。

**缺陷**

`代码事实`：local bootstrap 是局部 odom 模式；代码中存在提示：在线本地窗口不能直接切回 Earth mode，全局对齐通过 `map_to_odom` 表示。该行为在 `ic_gvins.cc:347-355` 和 `ic_gvins.cc:1769-1782` 附近有明确设计说明。

`基于代码的合理推断`：长期纯室内场景缺少绝对位置约束、回环约束、地图重识别、UWB/蓝牙/二维码/地标等室内锚点。单目视觉与 IMU 在长时间室内会累积漂移；如果纹理弱或动态多，漂移和跟踪丢失风险更高。

**分层评价**

- 系统级：具备无 GNSS 本地起步能力，但缺少室内全局闭环。
- 方法级：静止 IMU 初始化保守可靠，但只解决起步，不解决长期室内漂移。
- 思路级：把“启动”和“全局对齐”拆开是务实设计，但还不是完整室内定位方案。

**进步空间**

- 增加室内辅助观测接口，如 UWB、轮速、平面/高度约束、AprilTag 或已知地图匹配。
- 引入视觉回环或重定位模块，降低长期室内漂移。
- 为 local 模式定义更明确的状态输出和用户提示，避免误把局部 `odom` 当成全局定位。

**后续验证建议**

`需实验验证`：构造纯室内序列，关闭 GNSS 或注入起步 GNSS 缺失，比较 local bootstrap 成功率、初始化耗时、局部轨迹漂移、视觉健康状态、跟踪丢失次数。

### 2.3 室内外过渡场景

**现状**

`代码事实`：local bootstrap 后首次 GNSS 可触发全局对齐恢复段。`ic_gvins.cc:646` 在 local bootstrap 需要首个 GNSS-to-map alignment 时开启恢复段；`ic_gvins.cc:1058-1061` 在 local bootstrap 场景恢复确认后发出 `GLOBAL_ALIGNED` 事件。

`代码事实`：异步重定位节点可接收 `GlobalAligned`，并把恢复段节点送入可优化生命周期。`degraded_reloc_node.cc:317-330` 处理四类恢复事件；`reloc_segment_lifecycle.h:56-65` 将 `RecoveryConfirmed` 和 `GlobalAligned` 转为采集恢复锚点状态。

**优点**

`基于代码的合理推断`：室内外过渡正是 NcF 扩展较有针对性的场景。室内阶段保持连续 local odom；室外 GNSS 恢复后，在线轨迹不被强制跳变，全局修正由异步 `map_to_odom` 表达。

**缺陷**

`代码事实`：在线状态仍在 local/odom 轨迹中连续推进；异步 map 修正与在线估计器分离。`degraded_reloc_node.cc:493-498` 发布全局路径和 map-to-odom，而非直接重写主节点状态。

`基于代码的合理推断`：如果下游控制或导航只订阅 `/pose`/`/path` 而不使用 `map_to_odom` 或 `/global_path`，可能看不到全局修正。过渡期间 GNSS 恢复确认需要足够有效样本和基线，短暂露天或 GNSS 闪断可能不足以完成稳定对齐。

**分层评价**

- 系统级：在线连续性和全局修正解耦，适合实时系统。
- 方法级：恢复段和 raw GNSS anchor 设计较清晰，但对短窗口恢复和低质量 GNSS 仍敏感。
- 思路级：更像“局部里程计 + 异步全局校正”框架，而不是单一全局状态估计器。

**进步空间**

- 明确输出契约：下游应使用 `map -> odom -> base` 还是直接用 `/global_path`。
- 增加过渡场景的恢复置信度、锚点数量、基线长度和失败原因输出。
- 研究是否需要受控的在线状态重锚定机制，或继续坚持异步 map 校正但完善下游接口。

**后续验证建议**

`需实验验证`：使用“室内无 GNSS起步 -> 半遮蔽过渡 -> 室外恢复”的序列，统计 local bootstrap 成功率、`GLOBAL_ALIGNED` 触发时间、恢复锚点数量、`map_to_odom` 稳定性、恢复前后轨迹连续性。

### 2.4 室外天空大面积遮蔽场景

**现状**

`代码事实`：GNSS 静默超过 `gnss_timeout` 时，系统可打开恢复段。`ic_gvins.cc:655-672` 检查 GNSS timeout 并调用 `beginRecoverySegment`。配置中 KAIST urban38 默认 `gnss_timeout: 0.5`，i2Nav building00 和 full ablation 默认 `gnss_timeout: 5.0`。

`代码事实`：GNSS 大 innovation 会触发水平或垂直退化。`ic_gvins.cc:947-964` 使用预测天线位置和在线 GNSS 测量计算创新，并与水平/垂直阈值比较。

**优点**

`代码事实`：退化 GNSS 不一定直接进入在线因子，但会作为健康证据保留；恢复候选可生成 raw GNSS anchor 给异步 map 图优化。`ic_gvins.cc:1005-1024` 收集恢复对齐样本并可发出恢复锚点。

`基于代码的合理推断`：在城市峡谷、树荫、桥下、建筑边缘等天空遮蔽场景，系统可以通过 IMU/视觉维持短时连续性，并在 GNSS 恢复后用恢复段和异步重定位降低全局漂移影响。

**缺陷**

`代码事实`：GNSS 质量判断没有读取卫星数、RTK fix type、DOP、载噪比、残差、NLOS 标志等更丰富信息。现有输入主要是 `NavSatFix` 经纬高和 covariance。

`基于代码的合理推断`：天空遮蔽常导致多路径和 NLOS，covariance 未必准确反映真实误差。若接收机仍给出看似较小 covariance，innovation gate 可能要等到与预测状态差异足够大才触发退化，存在滞后。

**分层评价**

- 系统级：具备遮蔽检测、恢复段和全局校正框架。
- 方法级：timeout/innovation 简洁可调，但缺少 GNSS 原始质量建模。
- 思路级：把遮蔽视作“退化-恢复生命周期”是合理抽象，但观测质量维度还偏单薄。

**进步空间**

- 接入 `NavSatStatus`、RTK 状态、DOP、卫星数、接收机诊断或自定义 GNSS 质量消息。
- 增加连续遮蔽期间漂移预算和恢复可信度评估。
- 区分 GNSS 静默、水平 NLOS、垂直多路径、RTK fixed->float 切换等不同故障类型。

**后续验证建议**

`需实验验证`：在室外序列中注入 GNSS outage、covariance 膨胀、随机偏移和阶跃偏移，复用 `02_nc_timeout`、`03_nc_innovation_gate`、`04_nc_recovery_confirm`、`07_nc_async_reloc`、`08_nc_full`，统计恢复时间、恢复误差、误触发率和失败率。

### 2.5 视觉退化场景：动态、光线异常、弱纹理

**现状**

`代码事实`：视觉前端使用传统角点和 LK 光流。`tracking.cc:385-390`、`tracking.cc:487-493` 使用前向/反向 `calcOpticalFlowPyrLK`；`tracking.cc:647-651` 使用 `goodFeaturesToTrack` 和 `cornerSubPix`；`tracking.cc:690-756` 进行三角化和几何质量检查。

`代码事实`：NcF 扩展有视觉健康降权。优化中统计重投影残差数量和外点比例，`ic_gvins.cc:2303-2324` 更新视觉健康状态，并把 `visual_factor_std_scale_` 用于重投影因子标准差；`ic_gvins.cc:2850`、`ic_gvins.cc:3106` 将该缩放用于视觉因子。

**优点**

`基于代码的合理推断`：当动态、光照异常或弱纹理导致外点比例上升、有效残差不足时，系统可降低视觉因子权重，避免退化视觉过度主导融合。

**缺陷**

`代码事实`：前端没有语义动态物体剔除、光照质量评估、曝光补偿、运动模糊检测、图像增强或事件相机/深度输入等专门机制。视觉健康判断发生在优化阶段，更多是后验诊断与降权，而不是前端主动修复。

`基于代码的合理推断`：大面积动态物体、强反光、夜间过曝/欠曝、纹理重复或弱纹理时，LK 与角点前端可能先失败；健康降权能减害，但不一定能保证视觉约束继续可用。

**分层评价**

- 系统级：视觉失败有状态感知和权重反馈，但没有完整视觉恢复生命周期。
- 方法级：残差数量和外点比例是合理轻量指标，但不能区分动态、光照、模糊、弱纹理等原因。
- 思路级：当前更像“视觉质量后验降权”，不是“视觉退化主动鲁棒前端”。

**进步空间**

- 增加图像质量指标：亮度分布、对比度、模糊、饱和比例、特征空间分布。
- 增加动态剔除：语义 mask、光流一致性聚类、RANSAC 多模型或运动分割。
- 将视觉退化原因写入日志和健康面板，而不只输出状态。
- 在视觉退化时自适应调整特征提取、关键帧策略和重投影噪声。

**后续验证建议**

`需实验验证`：构造动态遮挡、亮度突变、模糊、低纹理片段，比较 `05_nc_visual_health` 与 baseline 的轨迹连续性、视觉 outlier ratio、`TRACK_LOST` 次数、关键帧密度和最终误差。

### 2.6 IMU 固定不稳或低成本 IMU

**现状**

`代码事实`：IMU 是传播骨架。`addNewImu` 中会检查时间间隔、数据有限性、角速度/比力幅值；异常时通过健康状态机给 IMU 区间设置 `noise_scale`。`ic_gvins.cc:423-449` 是入口和噪声缩放设置。

`代码事实`：默认配置中 `imu_max_angular_rate` 与 `imu_max_specific_force` 多为 0，意味着幅值上限诊断默认未真正启用；但 `imu_degraded_covariance_scale` 默认为可配置项，健康管理支持退化协方差缩放。

**优点**

`基于代码的合理推断`：IMU 时间间隔异常或明显异常输入可被识别为退化区间，并通过协方差膨胀降低对优化的支配力。对低成本 IMU 的噪声模型，也可通过 YAML 的 `imumodel` 调整。

**缺陷**

`代码事实`：IMU 固定不稳导致的是 IMU-相机、IMU-GNSS 杆臂或姿态外参随时间变化。当前代码有相机-IMU 外参估计开关，但没有把“固定松动”建模为时变外参或故障状态；GNSS 天线杆臂也是静态配置。

`基于代码的合理推断`：低成本 IMU 的温漂、尺度因子、震动、安装松动可能表现为长期系统性误差，单纯区间噪声膨胀不足以修正。若 IMU 与相机相对运动变化，视觉/惯性约束会出现结构性矛盾。

**分层评价**

- 系统级：对 IMU 输入异常有基本健康接口，但对安装可靠性没有专门链路。
- 方法级：预积分噪声膨胀是保守措施，不是低成本 IMU 完整误差建模。
- 思路级：系统仍然是 INS-centric，IMU 失稳属于高风险根因，应优先监测而非仅在优化中降权。

**进步空间**

- 启用并标定 `imu_max_angular_rate`、`imu_max_specific_force`，加入饱和、震动和静止不一致检测。
- 增加在线外参漂移或安装松动诊断，例如视觉重投影残差与 IMU 残差的相关异常。
- 对低成本 IMU 增加温度、 Allan 方差、随机游走和 bias 稳定性配置建议。
- 输出 IMU 健康历史和噪声缩放曲线，便于离线定位硬件问题。

**后续验证建议**

`需实验验证`：用低成本 IMU 数据或对高质量 IMU 注入噪声、bias 漂移、时间间隔抖动、短时饱和，比较轨迹误差、IMU 健康状态、预积分残差和优化稳定性。

### 2.7 部分传感器起步失效后恢复

**现状**

`代码事实`：GNSS 起步失效有处理路径。原始 GNSS 初始化需要可用 GNSS；NcF 开启且 `enable_local_bootstrap` 时，系统可在 GNSS 不可用时尝试 local bootstrap。`ic_gvins.cc:579-605` 在初始化阶段拒绝退化 GNSS 作为原始 IC 初始化；`ic_gvins.cc:1283` 触发 local bootstrap；`ic_gvins.cc:1749-1779` 完成本地初始化。

`代码事实`：可选航向输入仅在配置开启时使用，且要求 yaw 与标准差有效。`fusion_ros.cc:501-520` 处理航向消息；`heading_factor.h:14-29` 是 yaw 因子。

**优点**

`基于代码的合理推断`：GNSS 起步缺失但 IMU 静止条件满足时，系统可以先建立局部 odom，后续 GNSS 通过恢复段和异步重定位完成全局对齐。这对机器人从室内、车库、遮蔽区域起步有实际意义。

**缺陷**

`代码事实`：IMU 是必需传播源，没有看到缺 IMU 起步或 IMU 后续恢复的完整替代模式。视觉起步失败主要表现为 `TRACK_LOST` 和关键帧逻辑，缺少专门的“图像恢复后重初始化生命周期”。GNSS 恢复路径较完整，视觉和 IMU 的恢复路径相对弱。

`基于代码的合理推断`：系统对“GNSS 起步失效后恢复”支持较好；对“相机起步失效后恢复”“IMU 起步异常后恢复”“多传感器组合起步失效”支持有限。尤其 IMU 作为 INS-centric 主干，失效时系统会很脆弱。

**分层评价**

- 系统级：起步恢复能力主要围绕 GNSS 设计。
- 方法级：local bootstrap 和异步 map 校正构成完整 GNSS 恢复闭环；视觉/IMU 恢复闭环不足。
- 思路级：当前 NcF 名义上面向多传感器健康，但最成熟的是 GNSS 退化恢复。

**进步空间**

- 为相机输入增加缺帧、恢复、重初始化和特征质量冷启动策略。
- 为 IMU 异常起步增加明确失败状态与用户提示，避免传播错误状态继续扩大。
- 将各传感器起步状态抽象成统一 lifecycle，而不仅是运行期健康状态。

**后续验证建议**

`需实验验证`：分别构造 GNSS 起步缺失、图像起步缺失、航向起步缺失、短时 IMU 异常、GNSS 后续恢复等序列，统计初始化成功率、初始化耗时、恢复事件、恢复后误差和失败原因。

## 3. 消融与验证基础

`代码事实`：仓库已经有 i2Nav 消融配置与批跑脚本。`config/i2nav_ablation` 包含 00-08 方案；`scripts/run_i2nav_ablation_batch.sh` 会遍历 scheme、sequence 和 repeat，生成每次运行的 config、launch、metadata、status 和 run.log。

现有消融方案可按代码意图理解为：

- `00_ic_baseline`：NcF 关闭，尽量保留原始 IC-GVINS 行为。
- `01_nc_transparent`：NcF 开启但关键门控基本透明，用于检查扩展框架本身是否引入差异。
- `02_nc_timeout`：只强调 GNSS timeout。
- `03_nc_innovation_gate`：强调 GNSS innovation 门控。
- `04_nc_recovery_confirm`：加入恢复确认。
- `05_nc_visual_health`：加入视觉健康判断。
- `06_nc_local_bootstrap`：加入本地启动。
- `07_nc_async_reloc`：启用异步重定位。
- `08_nc_full`：完整 NcF 方案。

`基于代码的合理推断`：这些方案适合支撑后续从“单开关作用”到“完整系统作用”的逐步验证。但本文不运行这些脚本，也不比较任何数值。

## 4. 系统级、方法级、思路级总评

### 4.1 系统级

`代码事实`：NcF-GVINS 具有清晰的两节点结构：主节点在线估计，异步重定位节点做全局 map 校正。ROS 话题、消息、launch 和配置已有较完整链路。

`基于代码的合理推断`：系统级优势是工程解耦和容错结构清楚；系统级短板是输出语义较复杂，用户需要理解 `odom`、`map`、`map_to_odom`、`global_path` 的区别，否则容易误用。

### 4.2 方法级

`代码事实`：方法增量主要包括 GNSS 分轴准入、GNSS timeout/innovation/recovery、local bootstrap、异步 raw GNSS map anchor、视觉健康降权、IMU 噪声膨胀、可选航向约束。

`基于代码的合理推断`：最成熟的方法链条是 GNSS 退化与恢复；视觉和 IMU 健康更偏降权诊断，还没有同等级别的恢复闭环。

### 4.3 思路级

`基于代码的合理推断`：NcF-GVINS 的设计思路是从 INS-centric 的强主干出发，逐步引入非单一传感器中心的健康管理。这个思路适合复杂室外和室内外过渡，但若要覆盖纯室内、强动态、低成本硬件和多传感器失效，需要把健康状态从“权重调节”进一步扩展到“生命周期、替代观测和主动恢复”。

## 5. 后续实验建议

以下是后续验证路线，不属于本文已执行内容。

### 5.1 数据矩阵

- 常规室外：KAIST urban38/urban22、i2Nav street/playground/parking。
- 天空遮蔽：城市峡谷、树荫、桥下、楼边多路径片段，或在现有序列中注入 GNSS outage 和偏移。
- 室内：building 序列或自采室内序列，GNSS 全程缺失或 covariance 失效。
- 室内外过渡：室内无 GNSS 起步、半遮蔽走廊、室外 GNSS 恢复。
- 视觉退化：动态人车、弱纹理、强曝光、夜间、运动模糊。
- IMU 退化：低成本 IMU、松动安装、时间间隔抖动、bias 漂移、幅值饱和。
- 起步失效：GNSS 缺失、图像缺失、航向缺失、IMU 短时异常的组合。

### 5.2 消融组合

- 先跑 `00_ic_baseline` 与 `01_nc_transparent`，检查扩展框架本身是否改变正常场景行为。
- 对 GNSS 遮蔽跑 `02`、`03`、`04`、`07`、`08`，分离 timeout、innovation、恢复确认和异步重定位贡献。
- 对视觉退化跑 `05` 与 `08`，观察视觉降权对轨迹连续性和外点率的影响。
- 对室内起步和过渡跑 `06`、`07`、`08`，观察 local bootstrap 和 global alignment 的作用。

### 5.3 指标与观测文件

- 轨迹精度：`trajectory.csv`、`trajectory_unix.csv`、`global_path.csv`、`global_path_unix.csv`，使用 APE/RPE 与时间对齐评估。
- 连续性：轨迹中断、优化失败、`TRACK_LOST` 次数、关键帧间隔。
- 恢复能力：`RecoveryEvent` 时间、恢复段 ID、恢复确认耗时、恢复锚点数量、`map_to_odom` 变化幅度。
- 健康状态：GNSS horizontal/vertical、vision、IMU、heading 的状态切换次数和持续时间。
- 运行鲁棒性：批跑 status、run.log 错误原因、失败率、重复运行方差。
- 资源开销：`statistics.txt` 中优化耗时、边缘化耗时、特征数、外点数。

### 5.4 失败判据

- 初始化失败或长时间无法进入正常跟踪。
- 轨迹出现不可接受跳变，且下游 TF/Path 无法解释。
- GNSS 恢复后没有产生 `RECOVERY_CONFIRMED` 或 `GLOBAL_ALIGNED`。
- 视觉退化时外点率持续过高且没有有效降权。
- IMU 异常时噪声膨胀未触发，或触发后仍导致明显发散。
- 批跑中同一方案对同一序列重复运行不稳定。

## 6. 结论

`代码事实`：当前 NcF-GVINS 已在原始 IC-GVINS 基础上加入 GNSS 健康管理、恢复段、local bootstrap、异步重定位、视觉健康降权、IMU 区间噪声缩放和可选航向接口。

`基于代码的合理推断`：系统最适合有 IMU、单目图像和间歇可用 GNSS 的室外及室内外过渡任务。对纯室内、强视觉退化、低成本或松动 IMU、多传感器起步组合失效等场景，当前代码提供了部分基础，但还需要更多观测源、前端鲁棒性和统一传感器生命周期设计。

`需实验验证`：本文所有场景能力判断均来自静态代码分析。后续实验建议是下一阶段工作，不属于本次分析已执行内容。
