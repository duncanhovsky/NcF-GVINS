# 两阶段局部-全局初始化方法

## I. Problem Formulation

NcF-GVINS 将系统启动划分为局部里程计建立与全局地图对齐两个阶段。其目标是在 GNSS 不可用或不可置信时仍能建立连续、米制的在线里程计，并在后续 GNSS 恢复后通过显式的坐标变换把局部轨迹映射到全局 `map` 坐标系，而不是强行重写已经存在的滑窗状态。

设在线局部坐标系为 \(\mathcal{O}\)，全局 GNSS/local-NED 坐标系为 \(\mathcal{M}\)。时刻 \(k\) 的在线状态写为

$$
\mathbf{x}_k =
\left[
\mathbf{p}_k^{\mathcal O},
\mathbf{q}_k^{\mathcal O},
\mathbf{v}_k^{\mathcal O},
\mathbf{b}_{g,k},
\mathbf{b}_{a,k}
\right],
$$

其中 \(\mathbf{p}\)、\(\mathbf{v}\)、\(\mathbf{q}\) 分别表示位置、速度和姿态四元数，\(\mathbf{b}_g\)、\(\mathbf{b}_a\) 为陀螺和加速度计零偏。两阶段初始化估计的是

$$
\mathcal{T}_{\mathcal O \leftarrow \mathcal M}
=
\left(\mathbf{R}_z(\psi), \mathbf{t}\right),
\qquad
\mathbf{z}^{\mathcal O}
=
\mathbf{R}_z(\psi)\mathbf{z}^{\mathcal M}+\mathbf{t},
$$

即把全局 GNSS anchor 映射到在线 odom 轨迹的固定偏差变换。异步重定位节点随后可反向估计并发布从 odom 到 map 的全局修正。

## II. Stage I: Static Local Bootstrap

当 `nc_extension.enable_local_bootstrap` 置为 true 且系统未获得可信 GNSS 原点时，局部初始化器使用最近一段静止 IMU 窗口完成保守初始化。令 IMU 频率为 \(f_{\mathrm{imu}}\)，配置静止时长为 \(T_s\)，则所需窗口长度为

$$
N_s=\max\left(20,\left\lceil T_s f_{\mathrm{imu}}\right\rceil\right).
$$

系统从最近 \(N_s\) 个 IMU 增量
\(\{\Delta\boldsymbol{\theta}_i,\Delta\mathbf{v}_i\}_{i=1}^{N_s}\)
中检测零速度条件。若

$$
\mathrm{ZUPT}\left(
\{\Delta\boldsymbol{\theta}_i,\Delta\mathbf{v}_i\}_{i=1}^{N_s}
\right)=1,
$$

则以增量均值恢复等效角速度与比力：

$$
\bar{\boldsymbol{\omega}}
=
\frac{f_{\mathrm{imu}}}{N_s}
\sum_{i=1}^{N_s}\Delta\boldsymbol{\theta}_i,
\qquad
\bar{\mathbf{f}}
=
\frac{f_{\mathrm{imu}}}{N_s}
\sum_{i=1}^{N_s}\Delta\mathbf{v}_i .
$$

初始陀螺零偏取

$$
\hat{\mathbf{b}}_g=\bar{\boldsymbol{\omega}},
$$

初始 roll 和 pitch 由静止比力给出：

$$
\hat{\phi}=-\arcsin\frac{\bar f_y}{g},
\qquad
\hat{\theta}=\arcsin\frac{\bar f_x}{g}.
$$

若存在已标定且通过健康门控的外部航向观测，则使用其 yaw；否则局部 yaw 被定义为 gauge：

$$
\hat{\psi}=
\begin{cases}
\psi_{\mathrm{heading}}, & \text{calibrated heading is valid},\\
0, & \text{otherwise}.
\end{cases}
$$

因此局部初始状态为

$$
\mathbf{p}_0^{\mathcal O}=\mathbf{0},\quad
\mathbf{v}_0^{\mathcal O}=\mathbf{0},\quad
\mathbf{q}_0^{\mathcal O}=
\mathrm{Quat}(\hat{\phi},\hat{\theta},\hat{\psi}),\quad
\mathbf{b}_{g,0}=\hat{\mathbf{b}}_g,\quad
\mathbf{b}_{a,0}=\mathbf{0}.
$$

此时系统进入 local odom 模式，关闭 Earth-aware 预积分，保留普通重力向量
\(\mathbf{g}^{\mathcal O}=[0,0,g]^\top\)。该设计避免在缺失地理原点时构造错误的地球自转和曲率项。

## III. Stage II: GNSS-Origin Establishment and Global Alignment

若系统尚未 local bootstrap 且收到水平、垂直均可信的 GNSS，则沿用原始 IC-GVINS 的全局初始化路径：以 GNSS 的 BLH 作为地理原点，并由该位置计算重力。若 local bootstrap 已经生效，后续 GNSS 的作用发生改变：它只建立 \(\mathcal{M}\) 坐标系和全局对齐约束，不再修改已经运行的在线 local window。

当第一组可用 GNSS 到达时，系统保存

$$
\mathbf{o}_{\mathcal M} = \mathbf{z}^{\mathrm{BLH}}_0,
\qquad
\mathbf{z}^{\mathcal M}_i =
\mathrm{Global2Local}
\left(\mathbf{o}_{\mathcal M},\mathbf{z}^{\mathrm{BLH}}_i\right),
$$

其中 \(\mathbf{z}^{\mathcal M}_i\) 是无偏 raw GNSS anchor。在线预测天线位置为

$$
\hat{\mathbf{a}}^{\mathcal O}_i
=
\mathbf{p}^{\mathcal O}_i
+ \mathbf{R}\left(\mathbf{q}^{\mathcal O}_i\right)\boldsymbol{\ell},
$$

\(\boldsymbol{\ell}\) 为 GNSS 天线杆臂。恢复阶段收集匹配对

$$
\mathcal{P}
=
\left\{
\left(\mathbf{z}^{\mathcal M}_i,
\hat{\mathbf{a}}^{\mathcal O}_i\right)
\right\}_{i=1}^{n}.
$$

若样本数和水平基线满足可观测性条件，则以 2-D Procrustes 形式估计 yaw：

$$
\hat{\psi}
=
\mathrm{atan2}
\left(
\sum_i \tilde z_{x,i}\tilde a_{y,i}
- \tilde z_{y,i}\tilde a_{x,i},
\sum_i \tilde z_{x,i}\tilde a_{x,i}
+ \tilde z_{y,i}\tilde a_{y,i}
\right),
$$

其中
\(\tilde{\mathbf{z}}_i=\mathbf{z}^{\mathcal M}_i-\bar{\mathbf{z}}^{\mathcal M}\)，
\(\tilde{\mathbf{a}}_i=\hat{\mathbf{a}}^{\mathcal O}_i-\bar{\mathbf{a}}^{\mathcal O}\)。若水平基线不足，则 yaw 不可观，仅估计平移：

$$
\hat{\mathbf{t}}
=
\bar{\mathbf{a}}^{\mathcal O}
-
\mathbf{R}_z(\hat{\psi})\bar{\mathbf{z}}^{\mathcal M}.
$$

该变换作为恢复偏差进入在线 GNSS 因子：

$$
\mathbf{r}^{\mathrm{rec}}_g
=
\boldsymbol{\Lambda}_g
\left(
\mathbf{p}^{\mathcal O}_k
+\mathbf{R}(\mathbf{q}^{\mathcal O}_k)\boldsymbol{\ell}
-
\left[
\mathbf{R}_z(\hat{\psi})\mathbf{z}^{\mathcal M}_k+\hat{\mathbf{t}}
\right]
\right).
$$

其中 \(\boldsymbol{\Lambda}_g\) 是由 GNSS N/E/D 标准差和分轴有效性构成的信息矩阵。这样，恢复后的 GNSS 能约束在线 odom，但 raw GNSS anchor 本身仍保持无偏，继续供异步 map 图使用。

## IV. State Transition

两阶段初始化的状态机可概括为

$$
\mathrm{Startup} =
\begin{cases}
\mathrm{GlobalIC}, &
\mathrm{GNSS}_{h,v}\ \mathrm{valid\ before\ local\ bootstrap},\\
\mathrm{LocalOdom}, &
\mathrm{ZUPT}\ \mathrm{valid\ and\ local\ bootstrap\ enabled},\\
\mathrm{Wait}, &
\mathrm{otherwise}.
\end{cases}
$$

若配置 `startup_mode: local_static`，系统会在局部 gauge 建立前拒绝早到 GNSS 选择原始全局初始化路径。若配置为默认 auto，则可信 GNSS 优先，GNSS 不可用时才回退到 local bootstrap。

局部启动后的全局化不是把滑窗直接切换到 Earth mode。直接 handover 会要求同时变换活动位姿、地图点、边缘化先验和预积分模型，容易破坏优化一致性。因此系统采用如下输出契约：

$$
\mathrm{online\ odom}\ \text{remains continuous},
\qquad
\mathrm{global\ map}\ \text{is represented by}\ \mathcal{T}_{\mathcal M\leftarrow\mathcal O}.
$$

当 recovery 被确认时，主节点发布 `GLOBAL_ALIGNED` 事件，异步重定位节点基于 raw GNSS anchor 和在线相对运动估计 map 轨迹，并发布 `map -> odom` TF 以及 `/global_path`。

## V. Method Properties

1. 启动鲁棒性：GNSS 缺失时不再阻塞 VIO/GVINS 初始化，只要求静止 IMU 窗口满足零速检测。
2. 轨迹连续性：局部 odom 已运行后，GNSS 恢复不会直接造成在线状态跳变。
3. 全局一致性：raw GNSS 与 online-offset GNSS 分离，前者服务异步全局 map，后者服务连续在线 odom。
4. 可观测性约束：单个 GNSS anchor 不能观测 yaw；只有足够水平基线时才估计全局 yaw 修正。

## VI. Implementation Correspondence

本方法对应以下实现位置：

- `ic_gvins/ic_gvins/initialization/local_initializer.*`：静止 IMU 窗口、本地姿态和零偏估计。
- `ic_gvins/ic_gvins/ic_gvins.cc`：local bootstrap 激活、GNSS 原点建立、恢复段和 global-aligned 事件。
- `ic_gvins/ic_gvins/factors/recovery_gnss_factor.h`：恢复后在线 GNSS 因子。
- `ic_gvins/ROS/degraded_reloc_node.cc`：异步 map 图和 `map -> odom` 输出。
