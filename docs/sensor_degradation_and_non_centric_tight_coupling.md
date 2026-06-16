# 传感器退化评估与无中心紧耦合方法

## I. Problem Formulation

NcF-GVINS 在原始 INS-centric GVINS 的基础上引入传感器健康评估层。该层并不把某一传感器作为唯一的全局开关，而是为 GNSS 水平、GNSS 垂直、视觉、航向和 IMU 分别维护健康状态，并把状态映射为因子准入、测量标准差缩放或预积分噪声缩放。本文称其为无中心紧耦合：IMU 仍承担连续时间传播主干，但每类测量是否进入优化以及以多大权重进入优化，由其自身健康证据决定。

设传感器或测量分量集合为

$$
\mathcal{S}=\{g_h,g_v,c,\psi,i\},
$$

分别表示 GNSS 水平、GNSS 垂直、相机、外部航向和 IMU 区间。每个分量有状态

$$
s_k^m \in
\{\mathrm{UNAVAILABLE},\mathrm{ACTIVE},\mathrm{DEGRADED},\mathrm{RECOVERING}\},
\qquad m\in\mathcal{S}.
$$

健康评估输出三类量：

$$
\left(a_k^m,\gamma_k^m,s_k^m\right),
$$

其中 \(a_k^m\) 是是否准入优化的布尔量，\(\gamma_k^m\ge 1\) 是退化时的标准差或过程噪声缩放，\(s_k^m\) 是状态机标签。

## II. Modality-Neutral Health State Machine

对任一测量分量 \(m\)，令 \(v_k^m\in\{0,1\}\) 表示当前输入质量是否有效，\(N_m\) 表示恢复确认所需连续样本数，\(T_m\) 表示恢复确认所需最短持续时间。退化-恢复状态转移为

$$
s_k^m=
\begin{cases}
\mathrm{DEGRADED}, & v_k^m=0,\\
\mathrm{RECOVERING}, &
v_k^m=1,\ s_{k-1}^m\in\{\mathrm{DEGRADED},\mathrm{RECOVERING}\},\
C_k^m=0,\\
\mathrm{ACTIVE}, &
v_k^m=1,\ \left(s_{k-1}^m=\mathrm{ACTIVE}\ \mathrm{or}\ C_k^m=1\right),
\end{cases}
$$

其中恢复确认条件为

$$
C_k^m =
\left(n_k^m\ge N_m\right)
\land
\left(\Delta t_k^m\ge T_m\ \mathrm{or}\ T_m=0\right).
$$

当 \(v_k^m=0\) 时，连续恢复计数 \(n_k^m\) 清零；当状态处于 recovering 且输入有效时，\(n_k^m\) 增加。准入规则写为

$$
a_k^m =
\begin{cases}
1, & s_k^m=\mathrm{ACTIVE},\\
1, & s_k^m\in\{\mathrm{DEGRADED},\mathrm{RECOVERING}\}\ \land\ r_m=1,\\
0, & \mathrm{otherwise},
\end{cases}
$$

其中 \(r_m\) 表示退化时是否保留该模态。GNSS 和航向通常 \(r_m=0\)，即未确认恢复前不进入相应测量因子；视觉和 IMU 通常 \(r_m=1\)，即仍保留因子或传播，但增大其不确定性。

## III. GNSS Degeneration and Axis-Wise Admission

ROS 层从 `NavSatFix.position_covariance` 取 N/E/D 三轴标准差：

$$
\sigma_N=\sqrt{P_{4}},\qquad
\sigma_E=\sqrt{P_{0}},\qquad
\sigma_D=\sqrt{P_{8}}.
$$

给定阈值 \(\sigma_{\max}\)，输入有效性定义为

$$
v_h =
\mathbb{I}
\left(
0<\sigma_N<\sigma_{\max}
\land
0<\sigma_E<\sigma_{\max}
\land
\neg o_k
\right),
$$

$$
v_v =
\mathbb{I}
\left(
0<\sigma_D<\sigma_{\max}
\land
\neg o_k
\right),
$$

其中 \(o_k\) 为实验用 GNSS outage 标志。水平和垂直独立建模，使高度退化不会屏蔽可用的水平定位。

当在线状态已可预测天线位置时，系统进一步使用 innovation gate。预测天线位置为

$$
\hat{\mathbf{a}}_k
=
\mathbf{p}_k+\mathbf{R}(\mathbf{q}_k)\boldsymbol{\ell}.
$$

若已有恢复偏差
\((\mathbf{R}_z(\delta\psi),\delta\mathbf{t})\)，则用于在线因子的 GNSS 观测为

$$
\mathbf{z}^{\mathcal O}_k
=
\mathbf{R}_z(\delta\psi)\mathbf{z}^{\mathcal M}_{k,\mathrm{raw}}
+\delta\mathbf{t};
$$

否则
\(\mathbf{z}^{\mathcal O}_k=\mathbf{z}^{\mathcal M}_{k,\mathrm{raw}}\)。
水平和垂直创新为

$$
e_h = \left\|
\hat{\mathbf{a}}_{k,xy}-\mathbf{z}^{\mathcal O}_{k,xy}
\right\|_2,
\qquad
e_v =
\left|
\hat a_{k,z}-z^{\mathcal O}_{k,z}
\right|.
$$

若 \(e_h>\tau_h\)，水平 GNSS 进入退化；若 \(e_v>\tau_v\)，仅垂直行退化。准入后的 GNSS 残差为

$$
\mathbf{r}_g =
\boldsymbol{\Lambda}_g
\left(
\mathbf{p}_k+\mathbf{R}(\mathbf{q}_k)\boldsymbol{\ell}
-\mathbf{z}^{\mathcal O}_k
\right),
$$

其中

$$
\boldsymbol{\Lambda}_g=
\mathrm{diag}
\left(
\frac{a_h}{\sigma_N},
\frac{a_h}{\sigma_E},
\frac{a_v}{\sigma_D}
\right).
$$

若 \(a_h=1,a_v=0\)，GNSS 因子只提供水平二维约束；若 \(a_h=0\)，该 GNSS 不进入在线优化，但仍可作为恢复证据或异步 map anchor。

## IV. Vision, Heading, and IMU Health Evidence

视觉质量在第一次非线性优化后由重投影残差统计得到。令 \(N_r\) 为候选残差数量，\(N_o\) 为卡方剔除的外点数量，则

$$
\rho_o =
\begin{cases}
N_o/N_r, & N_r>0,\\
1, & N_r=0.
\end{cases}
$$

视觉有效性为

$$
v_c =
\mathbb{I}
\left(
N_r\ge N_{\min}
\land
\rho_o\le \rho_{\max}
\right).
$$

当视觉处于退化或恢复中时，重投影标准差被放大：

$$
\sigma_{\pi,k}' = \gamma_c\sigma_{\pi}.
$$

因此视觉因子仍可约束轨迹，但不会在低纹理、遮挡或高外点率下主导优化。

外部航向被视为已标定 yaw 观测，而非原始磁力计输入。设预测 yaw 为 \(\hat{\psi}_k\)，观测 yaw 为 \(\psi_k^{obs}\)，则 innovation 为

$$
e_{\psi} =
\left|
\mathrm{wrap}
\left(
\hat{\psi}_k-\psi_k^{obs}
\right)
\right|.
$$

若 \(e_{\psi}\le \tau_{\psi}\) 且通过恢复确认，则航向因子进入优化；否则拒绝或放大其标准差。

IMU 不能像外部测量一样被简单丢弃，因为它携带传播时间。系统检测 IMU 时间间隔与幅值：

$$
v_i =
\mathbb{I}
\left(
\Delta t_k \le 1.5\Delta t_{\mathrm{nom}}
\land
\|\Delta\boldsymbol{\theta}_k\|/\Delta t_k\le \omega_{\max}
\land
\|\Delta\mathbf{v}_k\|/\Delta t_k\le f_{\max}
\right).
$$

若幅值阈值未配置，则对应项不启用。退化 IMU 区间仍被积分，但预积分噪声矩阵放大：

$$
\mathbf{Q}_k'=\gamma_i\mathbf{Q}_k,
\qquad
\gamma_i\ge 1.
$$

这样可保留时序连续性，同时降低异常 IMU 对滑窗优化的支配作用。

## V. Non-Centric Tightly Coupled Objective

NcF-GVINS 的优化目标可写为

$$
\min_{\mathcal{X}}
\left(
\sum_{(i,j)}
\left\|
\mathbf{r}^{imu}_{ij}
\right\|_{\mathbf{\Sigma}_{ij}^{imu}(\gamma_i)^{-1}}^2
+
\sum_{\ell}
\rho
\left(
\left\|
\mathbf{r}^{vis}_{\ell}
\right\|_{\mathbf{\Sigma}_{\ell}^{vis}(\gamma_c)^{-1}}^2
\right)
+
\sum_{k}
\rho
\left(
\left\|
\mathbf{r}^{gnss}_{k}
\right\|_{\mathbf{\Sigma}_{k}^{gnss}(a_h,a_v)^{-1}}^2
\right)
+
\sum_{k}
\rho
\left(
\left\|
r^{heading}_{k}
\right\|_{\sigma_{\psi,k}^{-2}}^2
\right)
\right),
$$

其中 \(\rho(\cdot)\) 为鲁棒核。无中心性的关键不在于去除 IMU 传播，而在于每类因子都有独立的健康证据、状态机和权重映射：

$$
\mathrm{factor\ admission}
=
f_m
\left(
v_k^m,
s_{k-1}^m,
n_k^m,
\Delta t_k^m
\right),
\qquad
m\in\mathcal{S}.
$$

因此，GNSS 高度退化、视觉退化、IMU 异常或航向不可信都不会通过单一中心开关使整个融合链路失效。健康的测量分量仍可进入同一个滑窗因子图，并与 INS 传播共同约束状态。

## VI. Recovery Coupling

当 GNSS 水平状态从 ACTIVE 进入 DEGRADED 时，系统开启新的 recovery segment。恢复期中，满足输入质量和 innovation 的 GNSS 不立即改变在线轨迹，而是先作为恢复候选：

$$
\left(
\mathbf{z}^{\mathcal M}_{k,\mathrm{raw}},
\hat{\mathbf{a}}^{\mathcal O}_k
\right)
\in \mathcal{P}_{seg}.
$$

只有当健康状态机确认恢复且恢复偏差
\((\mathbf{R}_z(\delta\psi),\delta\mathbf{t})\)
有效时，GNSS 才重新进入在线紧耦合因子。未确认恢复但几何上有效的 anchor 可发送给异步 map 图，以避免在线 odom 跳变。

## VII. Implementation Correspondence

本方法对应以下实现位置：

- `ic_gvins/ic_gvins/health/sensor_health_manager.h`：统一健康状态机、恢复确认和退化权重。
- `ic_gvins/ROS/fusion_ros.cc`：GNSS 协方差读取、outage 标志和 recovery 消息发布。
- `ic_gvins/ic_gvins/ic_gvins.cc`：GNSS innovation、视觉残差统计、航向门控、IMU 诊断和因子准入。
- `ic_gvins/ic_gvins/factors/gnss_factor.h`：GNSS 水平/垂直分轴信息矩阵。
- `ic_gvins/ic_gvins/factors/recovery_gnss_factor.h`：恢复偏差下的在线 GNSS 因子。
- `ic_gvins/ic_gvins/preintegration/*`：IMU 噪声缩放进入预积分协方差传播。
