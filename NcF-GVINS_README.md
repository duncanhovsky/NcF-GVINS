# NcF-GVINS

NcF-GVINS 是基于 [IC-GVINS](README.md) 的 GNSS-Visual-Inertial 融合定位系统。原始 IC-GVINS 以 INS 为核心，将 IMU、单目视觉和 GNSS-RTK 在统一世界坐标系下进行紧耦合优化；本仓库在此基础上加入了 NcF/NC-IC 扩展，重点补充了传感器健康状态管理、GNSS 退化-恢复流程、可选本地里程计启动，以及独立于在线估计器的异步全局重定位节点。

从代码结构看，系统由两个主要 ROS 可执行程序组成：

| 节点 | 可执行文件 | 作用 |
| --- | --- | --- |
| 主融合节点 | `ic_gvins_ros` | 订阅 IMU、GNSS、图像和可选航向观测，运行在线 GVINS 滑窗优化，发布平滑的在线 `odom` 轨迹，并在 NcF 扩展开启时发布重定位数据包。 |
| 异步重定位节点 | `ic_gvins_reloc_node` | 订阅主节点发布的 `recovery_frame` 和 `recovery_event`，使用在线相对运动与无偏 raw GNSS 锚点构建 4DoF map 图优化，发布全局校正后的 `map` 轨迹和 `map -> odom` 变换。 |

主节点的实现入口在 `ic_gvins/ROS/fusion_ros.cc`，异步重定位节点在 `ic_gvins/ROS/degraded_reloc_node.cc`。生成消息定义位于 `ic_gvins/msg/RecoveryFrame.msg` 和 `ic_gvins/msg/RecoveryEvent.msg`。

## 1. 主要特性

- 继承 IC-GVINS 的 INS-centric GNSS/视觉/惯性紧耦合框架。
- 支持单目相机、IMU、GNSS-RTK 标准 ROS bag 输入。
- NcF 扩展将 GNSS 水平与垂直方向分开做健康判定，坏高度不会直接屏蔽可用的水平定位。
- 支持有限 GNSS outage 注入：`gnssoutagetime` 到 `gnssoutageendtime` 之间将 GNSS 标记为退化，用于验证恢复逻辑。
- 主节点保持在线 `odom` 轨迹连续，异步重定位节点单独输出全局校正的 `map` 轨迹，避免重定位阻塞实时估计。
- 支持无 GNSS 启动的本地 bootstrap：静止 IMU 满足条件后先建立局部 odom，后续 GNSS 通过恢复/重定位管线完成全局对齐。
- 预留可选磁航向接口，消息类型为 `geometry_msgs/PoseWithCovarianceStamped`，当前 KAIST/i2Nav 配置默认关闭。

## 2. 依赖环境

推荐环境与原始 IC-GVINS 基本一致：

- Ubuntu 18.04 + ROS Melodic，或 Ubuntu 20.04 + ROS Noetic。
- CMake >= 3.10，C++14 编译器，建议 `gcc >= 8` 或 `clang >= 6`。
- Ceres Solver 2.0/2.1。
- Eigen >= 3.3.7、OpenCV >= 3.2、yaml-cpp、glog、TBB、Boost。
- ROS 依赖：`roscpp`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`std_msgs`、`tf2_ros`、`message_generation`、`message_runtime`。

常用 apt 依赖安装示例：

```bash
sudo apt install build-essential cmake libeigen3-dev libopencv-dev \
    libyaml-cpp-dev libgoogle-glog-dev libtbb-dev libboost-filesystem-dev
```

Ceres 若系统源版本不满足，建议按 Ceres 官方方式从源码安装。

## 3. 编译

保持本仓库的目录结构，不要只拷贝 `ic_gvins` 子目录，因为 launch 文件会通过 `$(find ic_gvins)/../config` 查找配置文件。

```bash
mkdir -p ~/ncf_gvins_ws/src
cd ~/ncf_gvins_ws/src

# 将本仓库放到 src 下，例如：
# git clone <your_repo_url> NcF-GVINS
# 或者把已有的 NcF-GVINS 目录复制/软链接到这里

cd ~/ncf_gvins_ws
catkin_make -j8 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

如果需要指定 gcc-8：

```bash
catkin_make -j8 -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```

## 4. 输入与输出

### 4.1 输入 ROS 消息

主节点从私有参数读取话题名，代码默认值如下：

| 传感器 | ROS 消息 | 默认话题 | 说明 |
| --- | --- | --- | --- |
| IMU | `sensor_msgs/Imu` | `/imu0` | 代码按相邻消息时间差把角速度/加速度积分成 `dtheta/dvel`；IMU 坐标系要求为前-右-下。 |
| GNSS | `sensor_msgs/NavSatFix` | `/gnss0` | 使用 `latitude/longitude/altitude` 和 `position_covariance`。代码中协方差索引按 N/E/D 使用：`[4] -> N`、`[0] -> E`、`[8] -> D`。 |
| Camera | `sensor_msgs/Image` | `/cam0` | 支持 `mono8`、`bgr8`、`rgb8`；当前系统按单目相机处理。 |
| 可选航向 | `geometry_msgs/PoseWithCovarianceStamped` | `/mag_heading` | 仅当 YAML 中 `nc_extension.use_magnetic_heading: true` 时订阅；yaw 来自姿态四元数，标准差来自 `covariance[35]`。 |

### 4.2 主节点发布的话题

`ic_gvins_ros` 使用普通 `ros::NodeHandle` 发布如下全局话题：

| 话题 | 类型 | 坐标系 | 说明 |
| --- | --- | --- | --- |
| `/pose` | `nav_msgs/Odometry` | NcF 开启时为 `odom`，NcF 关闭时为 `world` | 在线融合位姿。 |
| `/path` | `nav_msgs/Path` | 同上 | 在线融合轨迹。 |
| `/tracking` | `sensor_msgs/Image` | 同上 | 特征跟踪可视化图像。 |
| `/current` | `sensor_msgs/PointCloud` | 同上 | 当前滑窗地图点。 |
| `/fixed` | `sensor_msgs/PointCloud` | 同上 | 被边缘化/固定的地图点。 |
| `/recovery_frame` | `ic_gvins/RecoveryFrame` | `odom` | 给异步重定位节点的在线状态快照和 raw GNSS 锚点。 |
| `/recovery_event` | `ic_gvins/RecoveryEvent` | `odom` | 退化开始、恢复确认、段关闭、全局对齐事件；该话题为 latched。 |

`/recovery_frame` 和 `/recovery_event` 只有在 YAML 中同时满足 `nc_extension.enabled: true` 与 `nc_extension.enable_async_relocator: true` 时才会由主节点发布。

### 4.3 异步重定位节点发布的话题

`ic_gvins_reloc_node` 订阅 `/recovery_frame` 和 `/recovery_event`，并发布：

| 话题/TF | 类型 | 说明 |
| --- | --- | --- |
| `/global_path` | `nav_msgs/Path` | 全局校正后的 map 轨迹。该节点只有在退化段恢复确认或全局对齐后才开始优化并发布。 |
| `/map_to_odom` | `geometry_msgs/TransformStamped` | 当前 map 到在线 odom 的校正变换。 |
| TF `map -> odom` | `tf2` | 与 `/map_to_odom` 相同的变换，便于 RViz 将平滑在线轨迹显示到全局 map 下。 |

注意：当前异步重定位节点会把 `/global_path` 的最新完整轨迹写入 `global_path.csv` 和 `global_path_unix.csv`。如果还需要离线查看 TF 或完整 ROS 话题，可运行时录制：

```bash
rosbag record /global_path /map_to_odom /tf
```

### 4.4 输出文件

主节点会根据 YAML 中的 `outputpath` 创建输出目录，并在 `is_make_outputdir: true` 时追加形如 `TYYYYMMDDhhmmss` 的时间戳子目录。输出包括：

| 文件 | 内容 |
| --- | --- |
| `gvins.nav` | 导航结果。全局初始化时为纬度、经度、高程、速度、姿态；本地 bootstrap 时位置为米制局部 odom。 |
| `trajectory.csv` | TUM 风格轨迹：`time p_x p_y p_z q_x q_y q_z q_w`。时间为 GPS week second。 |
| `trajectory_unix.csv` | 与 `trajectory.csv` 相同的在线轨迹，第一列时间为 Unix time。 |
| `global_path.csv` | 异步重定位 `/global_path` 的 8 列 TUM 风格轨迹，第一列时间为 GPS week second。 |
| `global_path_unix.csv` | 与 `global_path.csv` 相同的全局校正轨迹，第一列时间为 Unix time。 |
| `mappoint.txt` | 被保存的地图点。 |
| `statistics.txt` | 滑窗长度、特征数、重投影误差、优化耗时、外点统计等。 |
| `extrinsic.txt` | 在线估计的相机-IMU 外参和时间延迟。 |
| `IMU_ERR.bin` | IMU 误差估计二进制文件。 |
| `gvins.yaml` | 本次运行使用的配置备份。 |

代码使用 `boost::filesystem::create_directory`，不会递归创建父目录，也不会自动展开 `~`。因此建议把 `outputpath` 改成绝对路径，并提前创建父目录，例如 `/home/your_name/result/kaist_urban38/NcF-GVINS`。

## 5. 启动方式

所有 launch 都带有 `enable_nc_reloc` 参数，用来控制是否启动异步重定位节点。它只控制第二个节点是否启动；主节点是否发布重定位消息由 YAML 的 `nc_extension.enable_async_relocator` 决定。通常两者都保持为 `true`。

### 5.1 通用启动

通用 launch 默认使用 `/imu0`、`/gnss0`、`/cam0`：

```bash
source ~/ncf_gvins_ws/devel/setup.bash
roslaunch ic_gvins ic_gvins.launch \
    configfile:=/absolute/path/to/your/gvins.yaml \
    enable_nc_reloc:=true
```

然后播放 rosbag：

```bash
rosbag play /absolute/path/to/your_dataset.bag
```

如果你的话题名不同，可以复制一份 `ic_gvins/launch/ic_gvins.launch` 后修改：

```xml
<param name="imu_topic" value="/your/imu"/>
<param name="gnss_topic" value="/your/gnss"/>
<param name="image_topic" value="/your/image"/>
```

也可以直接用 `rosrun` 指定私有参数：

```bash
rosrun ic_gvins ic_gvins_ros \
    _configfile:=/absolute/path/to/your/gvins.yaml \
    _imu_topic:=/your/imu \
    _gnss_topic:=/your/gnss \
    _image_topic:=/your/image
```

若手动启动异步重定位节点：

```bash
rosrun ic_gvins ic_gvins_reloc_node \
    _relative_position_std:=0.15 \
    _relative_yaw_std_deg:=1.0 \
    _relative_reference_interval:=0.5 \
    _maximum_nodes:=5000
```

### 5.2 只运行在线融合，不运行异步重定位

只关闭异步 map 校正，但保留 NcF 健康管理：

```bash
roslaunch ic_gvins ic_gvins.launch \
    configfile:=/absolute/path/to/gvins.yaml \
    enable_nc_reloc:=false
```

同时把 YAML 中设为：

```yaml
nc_extension:
    enabled: true
    enable_async_relocator: false
```

如果要尽量回到原始 IC-GVINS 的 GNSS 输入逻辑，用于 baseline 对比，则把：

```yaml
nc_extension:
    enabled: false
```

此时主节点可视化坐标系会回到 `world`，不会发布重定位消息。

## 6. 数据集使用

### 6.1 KAIST Urban

仓库提供了 KAIST 专用 launch：

```bash
roslaunch ic_gvins ic_gvins_kaist.launch \
    configfile:=$(rospack find ic_gvins)/../config/gvins-kaist-urban38.yaml \
    enable_nc_reloc:=true
```

默认话题与参数来自 `ic_gvins/launch/ic_gvins_kaist.launch`：

| 输入 | 话题 |
| --- | --- |
| IMU | `/imu/data_raw` |
| GNSS | `/gps/fix` |
| 左目图像 | `/stereo/left/image_raw` |

该 launch 默认配置文件是 `config/gvins-kaist-urban38.yaml`。播放 urban38：

```bash
rosbag play /path/to/urban38.bag
```

如果运行 urban22，使用同一个 KAIST launch，但替换配置：

```bash
roslaunch ic_gvins ic_gvins_kaist.launch \
    configfile:=$(rospack find ic_gvins)/../config/gvins-kaist-urban22.yaml \
    enable_nc_reloc:=true
```

然后播放对应 rosbag：

```bash
rosbag play /path/to/urban22.bag
```

当前仓库中实际提供的 KAIST 配置为 `gvins-kaist-urban38.yaml` 和 `gvins-kaist-urban22.yaml`。如果要跑其他 KAIST 序列，例如 urban39，建议复制最接近的 KAIST 配置，检查并修改 `outputpath`、相机内外参、`imudatarate`、GNSS 频率相关的 `gnss_timeout`，以及是否需要估计外参和时间延迟。

KAIST 配置要点：

| 配置文件 | 图像频率 | IMU 频率 | GNSS 频率 | 默认重定位参数 |
| --- | --- | --- | --- | --- |
| `gvins-kaist-urban38.yaml` | 30 Hz | 100 Hz | 10 Hz | `relative_position_std=0.20`，`relative_yaw_std_deg=1.5` |
| `gvins-kaist-urban22.yaml` | 20 Hz | 110 Hz | 5 Hz | 同 KAIST launch，可按需要调整 |

### 6.2 i2Nav-Robot / building00

仓库提供了 i2Nav 专用 launch：

```bash
roslaunch ic_gvins ic_gvins_i2nav.launch \
    configfile:=$(rospack find ic_gvins)/../config/gvins-i2nav-building00.yaml \
    enable_nc_reloc:=true
```

默认话题：

| 输入 | 话题 |
| --- | --- |
| IMU | `/adi/adis16465/imu` |
| GNSS | `/gnss/fix` |
| 左目图像 | `/avt_camera/left/image` |

播放数据：

```bash
rosbag play /path/to/building00.bag
```

`config/gvins-i2nav-building00.yaml` 是当前 i2Nav building00 的推荐配置，使用 AVT 左相机作为单目输入，IMU 频率 200 Hz，GNSS 频率按 10 Hz 配置，默认异步重定位相对约束为：

```xml
<param name="relative_position_std" value="0.10"/>
<param name="relative_yaw_std_deg" value="1.0"/>
<param name="relative_reference_interval" value="0.5"/>
<param name="maximum_nodes" value="5000"/>
```

仓库中还保留了 `config/gvins-i2nav-robot.yaml`，可作为旧 i2Nav-Robot 数据或自定义机器人数据的起点。使用时建议复制为新文件，至少检查：

- `outputpath` 是否为绝对路径。
- 图像话题是否为 raw `sensor_msgs/Image`。如果 bag 只有 compressed 图像，可用 `image_transport` 转换为 raw。
- GNSS 话题名是否为 `/gnss/fix` 或 `/novatel/oem7/fix`，以实际 bag 为准。
- 相机内外参、`td_b_c`、天线杆臂 `antlever` 是否与设备标定一致。

若需要把 compressed 图像转 raw，可另开终端：

```bash
rosrun image_transport republish compressed raw \
    in:=/avt_camera/left/image \
    out:=/avt_camera/left/image
```

### 6.3 自采数据

自采数据需要满足：

- IMU、图像、GNSS 时间戳已经同步，ROS header 时间有效。
- IMU 坐标系为前-右-下；若设备输出为其他坐标系，需要预处理转换。
- 图像为 `mono8`、`bgr8` 或 `rgb8`。
- GNSS 的 `NavSatFix.position_covariance` 需要提供合理数值，否则健康门控会把 GNSS 视为不可用或退化。
- `cam0` 中内参、畸变、分辨率、`q_b_c`、`t_b_c`、`td_b_c` 与实际标定一致。
- `antlever` 为 GNSS 天线在 IMU body 坐标系下的杆臂，单位为米。

推荐流程：

1. 复制 `config/gvins.yaml` 为自己的配置文件。
2. 修改 `outputpath`、`imudatarate`、`antlever`、`imumodel` 和 `cam0`。
3. 按数据频率设置 `nc_extension.gnss_timeout`，例如 10 Hz GNSS 可设 0.5 s，5 Hz 可设 1.0 s，1 Hz 可设 2.0 s。
4. 复制并修改 launch 中的三个输入话题。
5. 先短时间播放 bag，确认 `/pose`、`/path`、`/tracking` 有输出，再打开完整序列。

## 7. NcF 关键配置说明

`nc_extension` 是本仓库相对原始 IC-GVINS 的主要新增配置段：

| 参数 | 作用 |
| --- | --- |
| `enabled` | 总开关。`false` 时尽量保持原始 IC-GVINS 行为。 |
| `enable_async_relocator` | 主节点是否发布 `RecoveryFrame/RecoveryEvent`。 |
| `gnss_timeout` | GNSS 静默多久后触发退化段，单位秒。 |
| `gnss_horizontal_innovation_threshold` | 水平 GNSS 创新阈值，超过后视为水平退化。 |
| `gnss_vertical_innovation_threshold` | 垂直 GNSS 创新阈值，超过后视为垂直退化。 |
| `horizontal_recovery_confirm_samples` / `vertical_recovery_confirm_samples` | 恢复确认需要的连续有效样本数。 |
| `horizontal_recovery_confirm_duration` / `vertical_recovery_confirm_duration` | 恢复确认还需要满足的最短持续时间。 |
| `recovery_min_horizontal_baseline` | 估计恢复偏差时需要的最小水平基线。 |
| `recovery_max_yaw_deg` | 恢复时允许的最大 yaw 修正。 |
| `startup_mode` | `"auto"` 优先使用健康 GNSS 初始化；`"local_static"` 强制走本地静止启动。 |
| `enable_local_bootstrap` | 无 GNSS 起步时是否允许本地 odom 初始化。 |
| `local_static_duration` | 本地启动需要的静止 IMU 时长。 |
| `enable_height_bias` | 可选 Down 方向 GNSS 高程偏差模型，默认关闭。 |
| `use_magnetic_heading` | 是否订阅可选的已校准 yaw 观测，默认关闭。 |
| `vision_*`、`imu_*`、`heading_*` | 视觉、IMU、航向健康判定与退化协方差缩放参数。 |

GNSS outage 实验使用顶层配置：

```yaml
isusegnssoutage: true
gnssoutagetime: 123456.0
gnssoutageendtime: 123556.0
```

这里的时间是代码内部使用的 GPS week second。`gnssoutageendtime <= gnssoutagetime` 时会保留原始 IC-GVINS 的单边 outage 行为，即从 `gnssoutagetime` 之后一直视为退化。

## 8. 常见问题

### `Failed to open configuration file`

`configfile` 路径错误。建议使用绝对路径，或用：

```bash
$(rospack find ic_gvins)/../config/gvins-kaist-urban38.yaml
```

### `Failed to open outputpath`

当前代码不会递归创建父目录，也不会展开 `~`。把 YAML 中的 `outputpath` 改为绝对路径，并提前创建父目录。

### `/path` 有输出但 `/global_path` 没有输出

通常是异步重定位还没有被触发。检查：

- launch 参数 `enable_nc_reloc:=true`。
- YAML 中 `nc_extension.enabled: true`。
- YAML 中 `nc_extension.enable_async_relocator: true`。
- `/recovery_frame` 和 `/recovery_event` 是否存在。
- 数据中是否真的出现了 GNSS 退化后恢复，或本地启动后收到 GNSS 全局对齐事件。

### 图像不更新或跟踪为空

检查图像编码是否为 `mono8`、`bgr8` 或 `rgb8`，相机内参/分辨率是否与 bag 一致；compressed 图像需要先 republish 为 raw。

### GNSS 一直不进入优化

检查 `NavSatFix.position_covariance`。代码用协方差开方作为 N/E/D 标准差，并与 `gnssthreshold` 和 NcF innovation 阈值比较。协方差为 0、NaN 或过大都会导致 GNSS 被判为无效或退化。

## 9. 结果查看与评估

RViz 会由 launch 自动打开，配置文件为 `config/visualization.rviz`。重点看：

- `/path`：在线连续 odom 轨迹。
- `/global_path`：异步重定位后的全局 map 轨迹。
- TF `map -> odom`：全局校正变换。
- `/tracking`：特征跟踪情况。

### 9.1 使用 evo 评估

推荐先安装 evo：

```bash
pip install evo --upgrade
```

评估前需要准备一份参考轨迹 `reference.tum`，格式为 TUM 8 列：

```text
timestamp tx ty tz qx qy qz qw
```

主节点结果是输出目录中的 `trajectory.csv`。虽然扩展名是 `.csv`，代码实际写出的是空格分隔的 8 列 TUM 风格轨迹，时间戳为 GPS week second。因此参考轨迹也应使用同一时间基准；如果参考轨迹是 ROS Unix time，需要先转换或在 evo 中使用合适的时间偏移。

主节点结果的 APE 评估：

```bash
evo_ape tum reference.tum /path/to/NcF-GVINS/T20260601123000/trajectory.csv \
    -a -p --plot_mode xy --save_results main_ape.zip
```

主节点结果与参考轨迹可视化：

```bash
evo_traj tum /path/to/NcF-GVINS/T20260601123000/trajectory.csv \
    --ref reference.tum -p --plot_mode xy
```

异步重定位结果会自动写入输出目录中的 `global_path.csv` 和 `global_path_unix.csv`。如果还需要离线查看 `/global_path` 话题、`/map_to_odom` 或 TF，可运行时录制：

```bash
rosbag record -O async_reloc_result.bag /global_path /map_to_odom /tf
```

可以直接用 evo 查看异步重定位轨迹形状：

```bash
evo_traj bag async_reloc_result.bag /global_path -p --plot_mode xy
```

若要对异步重定位结果计算 APE，可直接使用输出目录中的 `global_path.csv` 或 `global_path_unix.csv`，根据参考轨迹时间基准选择 GPS week second 或 Unix time：

```bash
evo_ape tum reference.tum /path/to/NcF-GVINS/T20260601123000/global_path.csv \
    -a -p --plot_mode xy --save_results async_reloc_ape.zip
```

注意：`degraded_reloc_node.cc` 发布 `/global_path` 话题时仍使用 `ros::Time::now()` 作为 Path 时间戳；新增的 `global_path.csv` 使用每个恢复节点原始 GPS time，`global_path_unix.csv` 使用对应 Unix time，更适合严格 APE 评估。

常用 evo 参数：

- `-a`：进行 SE(3) 轨迹对齐，适合比较形状和相对误差。
- `--pose_relation trans_part`：只评估平移误差，GNSS/SLAM 轨迹常用。
- `--t_max_diff 0.05`：设置时间关联容差，可根据数据频率调整。
- `--save_results xxx.zip`：保存 APE 统计结果，便于后续 `evo_res` 汇总。

`scripts/` 下还提供了原始 IC-GVINS 的统计查看脚本，使用前需要把脚本里的 `path/...` 改成实际输出目录：

```bash
python scripts/show_statistics.py
python scripts/show_tracking.py
python scripts/show_extrinsic.py
```

## 10. 代码索引

| 文件 | 内容 |
| --- | --- |
| `ic_gvins/ROS/fusion_ros.cc` | ROS 主节点，读取话题和配置，发布在线结果及 recovery 消息。 |
| `ic_gvins/ROS/degraded_reloc_node.cc` | 异步重定位节点，构建 raw GNSS 锚点约束和相对运动约束，发布 `global_path` 与 `map -> odom`。 |
| `ic_gvins/ic_gvins/ic_gvins.cc` | GVINS 核心流程：初始化、IMU 传播、GNSS 接入、恢复段管理、滑窗优化。 |
| `ic_gvins/ic_gvins/health/sensor_health_manager.h` | NcF 健康状态机：`ACTIVE`、`DEGRADED`、`RECOVERING`、`UNAVAILABLE`。 |
| `ic_gvins/ic_gvins/initialization/local_initializer.*` | 无 GNSS 本地静止启动逻辑。 |
| `ic_gvins/ic_gvins/common/types.h` | NcF 新增数据结构，包括 `RecoveryFrameData`、`RecoveryEventData`、`GNSS` 健康字段等。 |
| `ic_gvins/launch/*.launch` | 通用、KAIST、i2Nav 启动文件。 |
| `config/*.yaml` | 数据集和系统参数配置。 |

## 11. License

本仓库继承 IC-GVINS 的 GPLv3 许可。原始 IC-GVINS 作者与引用信息见 [README.md](README.md)。
