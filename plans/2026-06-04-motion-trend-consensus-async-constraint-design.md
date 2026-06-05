# NcF-GVINS 多源运动趋势共识与异步约束摘要设计

## 1. 目标

本文设计两个后续能力：

1. 传感器健康管理从“传感器自身质量判断”扩展为“多源相对运动趋势共识仲裁”。
2. 异步重定位节点从单一在线 4DoF 相对边扩展为可接收 IMU、视觉、航向等约束摘要的恢复段图优化。

核心假设是：当某个位置或姿态分量同时存在不少于 3 个独立趋势来源，且同一时间最多只有 1 个来源退化时，可以用多数一致性识别离群传感器。

## 2. 现状边界

当前 `SensorHealthManager` 已有 GNSS horizontal/vertical、vision、heading、IMU 的状态机，但证据主要来自各自的输入质量、innovation 或残差统计。GNSS 使用 covariance、forced outage 和在线预测天线位置 innovation；视觉使用重投影残差数量和外点率；IMU 使用时间间隔和幅值诊断。

当前异步重定位节点 `degraded_reloc_node.cc` 使用两类约束：

- 主节点发布的在线 odom 相对 4DoF 边。
- 恢复段 raw GNSS anchor。

它没有保存主节点退化段中的视觉重投影约束、IMU 预积分约束或边缘化先验。

## 3. 非目标

- 不在第一阶段传输完整视觉 landmark、逆深度、重投影观测和 marginalization prior。
- 不假设两个及以上独立传感器同时严重退化仍可被可靠隔离。
- 不直接实现在线状态重锚定。重锚定需要单独处理滑窗状态、路标、先验和坐标系一致性。

## 4. 趋势共识仲裁

### 4.1 趋势观测模型

新增 `MotionTrendObservation`，表达一个传感器或约束源在短时间窗口内独立计算出的相对运动：

- `source_type`：GNSS、IMU_PREINTEGRATION、VISION_RELATIVE、HEADING、ONLINE_ODOM、WHEEL_ODOM 等。
- `independence_group`：用于防止把同源信息重复计票，例如 ONLINE_ODOM 与 IMU/VISION 有耦合，默认只能作为软参考。
- `time_i/time_j`、`node_i/node_j`。
- `delta_position`、`delta_yaw`，必要时扩展 roll/pitch。
- `component_mask`：horizontal、vertical、yaw、attitude。
- `std` 或 information diagonal。
- `quality_score`、`prior_health_state`。

所有趋势先转换到在线 `odom` 局部坐标，并按分量比较：水平位移、垂直位移、yaw/姿态变化分开仲裁。

### 4.2 趋势来源

- GNSS：由相邻或固定间隔 raw GNSS local 点计算水平/垂直位移。优点是全局锚定；缺点是低频且易受 NLOS。
- IMU：由预积分给出相对位移、速度变化和姿态变化。优点是高频；缺点是 bias、饱和、安装松动会系统性漂移。
- 视觉：优先使用视觉前端或关键帧间视觉相对位姿摘要；若只能取优化后窗口相对位姿，则标记为与 ONLINE_ODOM 耦合的弱独立来源。
- 航向：只投票 yaw 分量。
- ONLINE_ODOM：当前融合结果趋势，可作为软参考，但不能和其内部来源同时满权计票。
- 轮速/UWB/地标：后续接入时按相同接口提供趋势。

### 4.3 仲裁规则

1. 对每个分量建立短窗，如 0.5 到 2.0 秒，按时间插值对齐。
2. 过滤不可比趋势：时间跨度不足、协方差无效、运动量过小、坐标系不明或同源重复。
3. 若独立来源数少于 3，回退到现有健康管理。
4. 使用 pairwise Mahalanobis residual、weighted median 或 RANSAC 找到最大一致集。
5. 若至少两个独立来源互相一致，且某一来源连续偏离一致集，则产生退化证据。
6. 退化证据不立即替代状态机，而是作为 `TrendHealthEvidence` 输入现有 `SensorHealthManager`。
7. 恢复同样需要连续窗口重新进入一致集，避免 GNSS 短时跳回或视觉偶然恢复导致抖动。

建议新增配置：

- `enable_motion_trend_consensus`
- `trend_window_duration`
- `trend_min_independent_sources`
- `trend_horizontal_threshold`
- `trend_vertical_threshold`
- `trend_yaw_threshold_deg`
- `trend_degrade_confirm_windows`
- `trend_recovery_confirm_windows`

### 4.4 与现有健康管理的关系

现有 `SensorHealthManager` 保留状态机、恢复确认和协方差缩放职责。趋势共识层只回答“某来源是否与多数独立趋势不一致”，并输出证据：

- `source_type`
- `component`
- `residual`
- `supporting_sources`
- `is_outlier`
- `confidence`

这样可以保持当前 GNSS recovery、视觉降权、IMU noise scale 的行为不被一次性推翻。

## 5. 异步约束摘要

### 5.1 为什么先传摘要

直接把主节点视觉重投影因子和 IMU 预积分因子传给异步节点，需要同步相机模型、特征 ID、landmark/逆深度、bias、外参、预积分协方差、边缘化先验和参数块生命周期。它会把异步节点变成第二个滑窗优化器，并带来双重计数风险。

更稳的第一阶段是传“约束摘要”：主节点仍负责把原始视觉/IMU 约束压缩为节点间相对边，异步节点只保存和优化这些边。

### 5.2 新消息建议

新增 `RecoveryConstraint.msg`：

- `std_msgs/Header header`
- `int32 segment_id`
- `uint64 node_i`
- `uint64 node_j`
- `float64 time_i`
- `float64 time_j`
- `int32 source_type`
- `geometry_msgs/Vector3 delta_position`
- `float64 delta_yaw`
- `float64[4] std`
- `float64 quality_score`
- `int32 health_state`
- `bool switchable`

可选再新增 `RecoveryConstraintArray.msg` 用于批量发布，减少 ROS 话题开销。

### 5.3 约束来源

- ONLINE_ODOM_RELATIVE：保留当前异步节点的相对 4DoF 因子。
- IMU_PREINTEGRATION_SUMMARY：来自相邻优化节点间的 IMU 预积分相对运动和协方差摘要。
- VISION_RELATIVE_SUMMARY：来自关键帧间视觉约束的相对位姿摘要、有效残差数和外点率。
- HEADING_RELATIVE：航向变化边，仅约束 yaw。

GNSS raw anchor 仍保持现有 `RecoveryFrame` 路径，不通过约束摘要替代。

### 5.4 异步图优化改动

异步节点新增约束缓存，按 `(segment_id, node_i, node_j, source_type)` 去重。构图时：

- raw GNSS anchor 仍提供全局位置约束。
- ONLINE_ODOM_RELATIVE 作为默认连续性边。
- IMU/VISION/HEADING 摘要边按健康状态和质量分数设置协方差。
- 对所有摘要边使用 Huber loss；对高风险来源可使用 switchable factor 或更大的协方差。
- 避免满权双重计数：如果 ONLINE_ODOM 已由 IMU 和视觉融合得到，则 IMU/VISION 摘要边不能与 ONLINE_ODOM 同时满权使用。初期建议 ONLINE_ODOM 降为弱连续先验，IMU/VISION 摘要边承担主要局部形状约束。

### 5.5 数据流

主节点：

1. 优化结束后、边缘化前，沿 `emitRecoveryFrames()` 相同节奏生成约束摘要。
2. 对退化段内 keyframe/GNSS time node 之间的相邻边发布 `RecoveryConstraintArray`。
3. 写入 source、std、health_state、quality_score。

异步节点：

1. 订阅 `recovery_constraint` 或 `recovery_constraints`。
2. 约束进入生命周期缓存，随 `RecoveryEvent` 打开、确认和关闭 segment。
3. `optimizeAndPublish()` 构造多源相对边 + GNSS anchor 图。
4. 输出仍为 `global_path`、`path_map`、`map_to_odom` 和 `reloc_segments`。

## 6. 分阶段实施

阶段 A：趋势共识只记录日志和健康面板诊断，不影响融合权重。目标是验证误报率。

阶段 B：趋势共识证据进入 `SensorHealthManager`，只影响 GNSS/vision/heading 的降权，不直接影响 IMU 传播。

阶段 C：新增 `RecoveryConstraint` 消息和异步节点缓存，但只接入 ONLINE_ODOM_RELATIVE，保持结果等价。

阶段 D：接入 IMU_PREINTEGRATION_SUMMARY 和 VISION_RELATIVE_SUMMARY，使用消融实验验证恢复段图优化是否更稳。

阶段 E：若摘要约束仍不足，再研究传输更接近原始的视觉/IMU 因子。

## 7. 验证方案

- 单元测试：趋势对齐、RANSAC/median 仲裁、最多一个离群源的隔离、同源去重。
- 回放测试：GNSS 阶跃偏移、IMU 短时跳变、视觉退化片段分别注入。
- 消融：现有 `07_nc_async_reloc` 与新增摘要约束版本比较 `global_path` APE/RPE、恢复锚点数量、`map_to_odom` 稳定性。
- 日志指标：每个趋势源的残差、投票结果、健康状态切换原因、异步边数量和优化残差。

## 8. 主要风险

- ONLINE_ODOM 与 IMU/视觉不是完全独立，计票时必须降权或标记同源。
- 低运动量窗口无法可靠判断趋势，必须设置最小位移/最小 yaw 门槛。
- GNSS 低频会导致趋势窗口和 IMU/视觉窗口不完全对齐。
- 摘要边协方差若过乐观，会让异步图过度相信退化段内部形状。
- 多个传感器同时退化时，多数投票假设失效，需要回退到保守降权。

## 9. 本轮实现状态

- 已新增纯 C++ `MotionTrendObservation` / `TrendHealthEvidence` 共识核心，并在 GNSS 健康判断前接入水平趋势证据；当独立趋势来源少于配置值时自动回退现有质量与 innovation 判定。
- 已新增 `RecoveryConstraint.msg`，主节点在发布恢复段 `RecoveryFrame` 的同时，为相邻图节点发布 `ONLINE_ODOM_RELATIVE` 摘要边。
- 异步重定位节点已订阅 `recovery_constraint`，按 `(segment_id, node_i, node_j, source_type)` 去重缓存；构图时优先使用摘要边，没有摘要边时保留原来的连续 odom 相对边。
- IMU_PREINTEGRATION_SUMMARY、VISION_RELATIVE_SUMMARY 和 HEADING_RELATIVE 的消息接口与异步图入口已经预留，后续可在主节点生成对应摘要后直接复用缓存和优化图路径。
