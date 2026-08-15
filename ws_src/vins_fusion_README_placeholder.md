# vins_fusion（ROS2 社区移植）

本目录存放 VINS-Fusion 的 ROS2 移植版。官方 [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) 仅支持 ROS1，这里使用社区移植版（目标 ROS2 Humble，部分原生支持 D435i）。

## 获取方式（二选一）

> 本项目目录当前**不是 git 仓库**，无法使用 `git submodule`，请直接 clone 到本目录。

### 方案 A（推荐，原生 D435i 支持）

```bash
cd ws_src
git clone https://github.com/fanhong-li/VINS-Fusion-ROS2-Humble.git vins_fusion
```

### 方案 B（通用 ROS2 移植）

```bash
cd ws_src
git clone https://github.com/zinuok/VINS-Fusion-ROS2.git vins_fusion
```

## 构建

```bash
cd /workspace
colcon build --symlink-install --packages-up-to vins
source install/setup.bash
```

> 必须用 `--packages-up-to vins`（连同依赖 `camera_models` 一起编）；`--packages-select vins` 只挑 vins 一个包，会因找不到依赖而失败。
> 依赖：`libeigen3-dev libopencv-dev libceres-dev`（`libceres-dev` 已补进 Dockerfile；**旧镜像需手动** `apt-get install -y libceres-dev`）。
> ⚠️ 此 fork 的 `estimator.cpp` 引用了 `ceres::CUDA`，apt 版 Ceres（无 CUDA）编译不过，已用 `#ifdef CERES_HAVE_CUDA` 保护，未定义时退化为 DENSE_SCHUR（详见 [`docs/02`](../docs/02_VINS-Fusion_ROS2集成.md) §8）。
> 若包名不是 `vins`，以 clone 后 `package.xml` 里的 `<name>` 为准。

## 运行（D435i 双目+IMU）

```bash
ros2 launch realsense2_camera rs_launch.py    # 终端 1（D435i，话题 /camera/infra1|2/image_rect_raw + /camera/imu）
ros2 run vins vins_node ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml   # 终端 2（在 /workspace 下）
rviz2 -d ws_src/vins_fusion/config/vins_rviz_d435i.rviz   # 终端 3（VINS 轨迹 /path + 双目红外 + 彩色）
```

> 配置文件真实路径为 `ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml`（注意是 `realsense_stereo_imu_config.yaml`，不是 `vins_rviz_d435i.yaml`）。
> ⚠️ 配置已按这台 D435i 实测调好：话题 `/camera/infra1|2/image_rect_raw` + `/camera/imu`，`imu: 1`，内外参按 realsense 出厂标定写入。**IMU 订阅 QoS 已在 `vins/src/rosNodeTest.cpp` 改成 `.best_effort()`**（realsense 的 IMU 是 BEST_EFFORT，reliable 会收不到），改动后需重新 `colcon build --packages-up-to vins`。

**关闭节点**（realsense 进程 comm 截断为 15 字符，用截断名）：

```bash
pkill -x vins_node
pkill -x realsense2_came     # 不是 realsense2_camera_node
pkill -x rviz2
```

具体配置与话题见 [`docs/02`](../docs/02_VINS-Fusion_ROS2集成.md)。

## 关键帧与地图点（已具备）

此 fork（fanhong-li）**已内置**关键帧相关 publisher（`vins/src/utility/visualization.cpp`），无需再补：

- `keyframe_pose`：`nav_msgs/Odometry`
- `keyframe_point`：`sensor_msgs/PointCloud`（关键帧稀疏点）

> 注意 fork 默认以**相对话题名**发布（`odometry`/`path`/`point_cloud`/`keyframe_pose`/`keyframe_point`，无 `/vins_fusion/` 前缀）。object_slam 需要 `/vins_fusion/keyframe_pose`（`PoseStamped`）与 `/vins_fusion/map_points` 时，做**话题重映射或类型适配**即可，不要重复添加 publisher。详见 [`docs/02`](../docs/02_VINS-Fusion_ROS2集成.md) §4。

## 许可

VINS-Fusion 及其移植版为 **GPLv3**，请遵守其许可条款。
