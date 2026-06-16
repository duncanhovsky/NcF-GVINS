# 异步重定位方法

## I. Problem Formulation

NcF-GVINS 的异步重定位节点将全局 map 校正从在线滑窗优化中解耦。在线主节点保持连续 odom 输出，并在边缘化前复制可成图的状态快照、raw GNSS anchor 和相对运动约束。异步节点订阅这些消息，在独立线程中构建 degraded segment 的 4DoF map 图，优化后发布 `/global_path`、`/path_map` 和 `map -> odom` 变换。

对每个异步节点 \(i\)，定义待优化变量为

$$
\mathbf{y}_i =
\left[
\mathbf{p}_i^{\mathcal M},
\psi_i^{\mathcal M}
\right],
$$

其中 \(\mathbf{p}_i^{\mathcal M}\in\mathbb{R}^3\) 为 map 坐标系位置，\(\psi_i^{\mathcal M}\) 为 yaw。roll 和 pitch 不在异步图中独立优化，而由在线 odom 姿态保留。这样图优化只修正全局平移和 yaw drift，避免重新求解完整 VIO/GVINS 滑窗。

## II. Inputs and Segment Lifecycle

主节点向异步节点提供三类消息：

1. `RecoveryFrame`：包含 `segment_id`、`node_id`、在线 odom 位姿、天线杆臂、raw GNSS local-NED anchor、N/E/D 标准差、健康状态和 revision。
2. `RecoveryEvent`：标记 `DEGRADED_START`、`RECOVERY_CONFIRMED`、`SEGMENT_CLOSED` 和 `GLOBAL_ALIGNED`。
3. `RecoveryConstraint`：紧凑相对运动摘要，默认来源为在线 odom relative，也可扩展为 IMU、视觉或航向摘要。

对每个 segment，生命周期为

$$
\mathrm{Idle}
\rightarrow
\mathrm{Capturing}
\rightarrow
\mathrm{CollectingRecoveryAnchors}
\rightarrow
\mathrm{ReadyToSolve}
\rightarrow
\mathrm{Solving}
\rightarrow
\mathrm{Done/Failed}.
$$

当接收到 `DEGRADED_START` 时，segment 开始捕获节点；当接收到 `RECOVERY_CONFIRMED` 或 `GLOBAL_ALIGNED` 时，节点继续收集恢复后的 GNSS anchor。若恢复 anchor 数达到 \(N_a\)，或恢复尾段超过设定时长且至少存在一个 anchor，则 segment 被加入待优化队列：

$$
\mathrm{Ready}
=
\left(
n_a\ge N_a
\right)
\lor
\left(
\Delta t_{\mathrm{tail}}\ge T_{\mathrm{tail}}
\land n_a>0
\right),
\qquad
|\mathcal{V}_{seg}|\ge 2.
$$

该队列由异步工作线程消费，不阻塞主融合线程。

## III. Relative-Motion Constraint

设在线 odom 位姿为
\((\mathbf{p}_i^{\mathcal O},\psi_i^{\mathcal O})\)。相邻节点或显式摘要边提供相对运动观测

$$
\Delta\mathbf{p}_{ij}^{\mathcal O}
=
\mathbf{R}_z(-\psi_i^{\mathcal O})
\left(
\mathbf{p}_j^{\mathcal O}-\mathbf{p}_i^{\mathcal O}
\right),
$$

$$
\Delta\psi_{ij}^{\mathcal O}
=
\mathrm{wrap}
\left(
\psi_j^{\mathcal O}-\psi_i^{\mathcal O}
\right).
$$

异步图中的 4DoF 相对残差为

$$
\mathbf{r}_{ij}^{rel}
=
\begin{bmatrix}
\mathbf{R}_z(-\psi_i^{\mathcal M})
\left(
\mathbf{p}_j^{\mathcal M}-\mathbf{p}_i^{\mathcal M}
\right)
-
\Delta\mathbf{p}_{ij}^{\mathcal O}
\\
\mathrm{wrap}
\left(
\psi_j^{\mathcal M}-\psi_i^{\mathcal M}
-\Delta\psi_{ij}^{\mathcal O}
\right)
\end{bmatrix}.
$$

若 `RecoveryConstraint` 已提供摘要边，异步节点按 `(segment_id,node_i,node_j,source_type)` 去重并采用最新 revision；否则退回到相邻 odom 节点构造相对边。标准差随时间间隔缩放：

$$
\sigma_{ij}
=
\sigma_0
\sqrt{
\max
\left(
\frac{t_j-t_i}{T_{ref}},
1
\right)
}.
$$

实现中等价地对位置和 yaw 标准差乘以该平方根尺度，从而避免长间隔 odom 边被赋予过强约束。

## IV. Raw GNSS Anchor Constraint

raw GNSS anchor 保持全局无偏，不包含在线恢复偏差。对具有 raw GNSS 的节点 \(i\)，设
\(\mathbf{g}_i^{\mathcal M}\) 为 GNSS local-NED 观测，
\(\boldsymbol{\ell}_i^{\mathcal O}\) 为在线姿态旋转后的天线杆臂。异步图估计的 map 姿态与在线 odom yaw 之间存在修正

$$
\delta\psi_i = \psi_i^{\mathcal M}-\psi_i^{\mathcal O}.
$$

天线杆臂在 map 图中的水平旋转为

$$
\boldsymbol{\ell}_i^{\mathcal M}
=
\mathbf{R}_z(\delta\psi_i)\boldsymbol{\ell}_i^{\mathcal O}.
$$

GNSS anchor 残差为

$$
\mathbf{r}_{i}^{g}
=
\begin{bmatrix}
m_h\frac{p_{i,x}^{\mathcal M}+\ell_{i,x}^{\mathcal M}-g_{i,x}^{\mathcal M}}{\sigma_{N,i}}\\
m_h\frac{p_{i,y}^{\mathcal M}+\ell_{i,y}^{\mathcal M}-g_{i,y}^{\mathcal M}}{\sigma_{E,i}}\\
m_v\frac{p_{i,z}^{\mathcal M}+\ell_{i,z}^{\mathcal O}-g_{i,z}^{\mathcal M}}{\sigma_{D,i}}
\end{bmatrix},
$$

其中 \(m_h,m_v\in\{0,1\}\) 分别为水平和垂直有效性掩码。若垂直 GNSS 退化，第三行置零；若水平退化，前两行置零。该因子刻意不使用 online offset，因此不会把在线 recovery 偏差反馈成全局 map bias。

## V. Segment Graph Optimization

异步重定位优化目标为

$$
\min_{\{\mathbf{y}_i\}}
\sum_{(i,j)\in\mathcal{E}_{rel}}
\rho_{rel}
\left(
\left\|
\mathbf{r}_{ij}^{rel}
\right\|_{\mathbf{\Sigma}_{ij}^{-1}}^2
\right)
+
\sum_{i\in\mathcal{A}_g}
\rho_g
\left(
\left\|
\mathbf{r}_{i}^{g}
\right\|_{\mathbf{\Sigma}_{g,i}^{-1}}^2
\right).
$$

其中 \(\mathcal{E}_{rel}\) 为相对运动边集合，\(\mathcal{A}_g\) 为包含 raw GNSS anchor 的节点集合。GNSS anchor 使用 Huber 鲁棒核；显式摘要相对边也使用 Huber 核。图变量初值来自在线 odom：

$$
\mathbf{p}_i^{\mathcal M,0}=\mathbf{p}_i^{\mathcal O},
\qquad
\psi_i^{\mathcal M,0}=\psi_i^{\mathcal O}.
$$

单个 GNSS anchor 无法观测全局 yaw，因此 yaw 需要至少两个具有方向信息的 anchor 和足够运动基线才稳定。实现中还要求 segment 内 GNSS anchor 数量不少于配置的 `min_recovery_anchors`，以避免短段或瞬时恢复造成不可靠 map 修正。

## VI. Output Map Correction

求解成功后，异步节点保存每个优化节点的 map 位姿。对 segment 最后一个节点 \(n\)，定义从 odom 到 map 的当前修正为

$$
\mathbf{R}_{\mathcal M\leftarrow\mathcal O}
=
\mathbf{R}_z
\left(
\psi_n^{\mathcal M}-\psi_n^{\mathcal O}
\right),
$$

$$
\mathbf{t}_{\mathcal M\leftarrow\mathcal O}
=
\mathbf{p}_n^{\mathcal M}
-
\mathbf{R}_{\mathcal M\leftarrow\mathcal O}
\mathbf{p}_n^{\mathcal O}.
$$

未被 segment 直接优化的后续 odom 节点使用最近一次可用修正映射到 map：

$$
\mathbf{p}^{\mathcal M}
=
\mathbf{R}_{\mathcal M\leftarrow\mathcal O}
\mathbf{p}^{\mathcal O}
+
\mathbf{t}_{\mathcal M\leftarrow\mathcal O},
$$

$$
\mathbf{q}^{\mathcal M}
=
\mathbf{R}_{\mathcal M\leftarrow\mathcal O}
\otimes
\mathbf{q}^{\mathcal O}.
$$

节点随后发布：

- `/global_path`：全局校正后的 map 轨迹。
- `/path_map`：与 `/global_path` 同源的 map 轨迹输出。
- `/map_to_odom`：`map` 父坐标系下 `odom` 子坐标系的 TF。
- `global_path.csv` 与 `global_path_unix.csv`：用于离线 APE/RPE 评估的 TUM 风格轨迹。

## VII. Method Properties

1. 在线连续性：主融合线程只发布 recovery 数据，不等待异步图优化，也不回写历史滑窗状态。
2. 无偏全局 anchor：异步 GNSS 因子使用 raw GNSS，避免在线 recovery offset 污染 map。
3. 段级优化：退化段、恢复确认和全局对齐事件由显式 lifecycle 管理，避免从高频状态快照中隐式推断。
4. 可扩展相对边：`RecoveryConstraint` 允许在线 odom、IMU、视觉、航向等摘要约束进入同一异步图；没有摘要时保留连续 odom 相对边作为 fallback。

## VIII. Implementation Correspondence

本方法对应以下实现位置：

- `ic_gvins/ROS/degraded_reloc_node.cc`：异步节点、4DoF 因子、GNSS anchor、segment 优化和全局路径输出。
- `ic_gvins/ROS/reloc_segment_lifecycle.h`：segment 生命周期与 ready 队列。
- `ic_gvins/ROS/recovery_constraint_summary.h`：相对运动摘要边缓存与 revision 去重。
- `ic_gvins/msg/RecoveryFrame.msg`、`RecoveryEvent.msg`、`RecoveryConstraint.msg`：主节点到异步节点的数据契约。
- `ic_gvins/ic_gvins/ic_gvins.cc`：边缘化前发布 recovery frame 和 relative constraint。
