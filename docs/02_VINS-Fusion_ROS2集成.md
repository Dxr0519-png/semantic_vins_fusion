# 02 · VINS-Fusion ROS2 集成

## 1. 目标

把 VINS-Fusion（视觉惯性里程计）接入 ROS2，输出物体级模块所需的**位姿、稀疏点云、关键帧**，并在动态场景下仍能稳定运行。

## 2. 选型

官方 [HKUST-Aerial-Robotics/VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) 仅支持 ROS1。本项目使用社区 ROS2 移植版（详见 [`ws_src/vins_fusion/README.md`](../ws_src/vins_fusion/README.md)）：

- 首选 `fanhong-li/VINS-Fusion-ROS2-Humble`（原生 D435i 支持）；
- 备选 `zinuok/VINS-Fusion-ROS2`。

## 3. 输入/输出话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 左目灰度 → VINS |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 右目灰度 → VINS |
| `/camera/imu` | `sensor_msgs/Imu` | IMU → VINS |
| `/odometry` | `nav_msgs/Odometry` | VINS → 外部（实测 ~15Hz） |
| `/path` | `nav_msgs/Path` | VINS → rviz 轨迹 |
| `/tf` | `tf2_msgs/TFMessage` | VINS → 坐标变换 `world→body`（本仓库已启用 pubTF） |
| `/point_cloud` | `sensor_msgs/PointCloud` | VINS → object_slam |
| `/keyframe_pose` | `nav_msgs/Odometry` | VINS → object_slam（**重映射/适配**） |
| `/keyframe_point` | `sensor_msgs/PointCloud` | VINS → object_slam（**重映射/适配**） |

> **实测核对**：此 fork 以**相对话题名**发布，vins 节点在根命名空间，所以实际话题是 `/odometry`、`/path`、`/point_cloud`、`/keyframe_pose`、`/keyframe_point`（**无 `/vins_fusion/` 前缀**）。若 object_slam 需要 `/vins_fusion/*`，给 vins 节点做话题重映射即可。

## 4. 关键帧与地图点暴露

物体级模块做"2D 分割 mask 与 3D 点关联"需要 VINS 的关键帧位姿和世界系 3D 点。**此 fork 已内置**对应 publisher（[vins/src/utility/visualization.cpp](../ws_src/vins_fusion/vins/src/utility/visualization.cpp)：publisher 声明 L22-23、创建 L48-49、`keyframe_pose` 发布 L475、`keyframe_point` 发布 L513）：

| fork 已有 | 类型 | 说明 |
|---|---|---|
| `keyframe_pose` | `nav_msgs/Odometry` | 关键帧位姿（Odometry 类型，非 PoseStamped） |
| `keyframe_point` | `sensor_msgs/PointCloud` | 关键帧稀疏点，可作 `map_points` |

因此**无需从零补代码**。object_slam 侧按以下方式对接：

- **话题重映射**：给 vins 节点加 remap 使其发布到 `/vins_fusion/*`，或直接改 object_slam 订阅的话题名；
- **类型适配**：若必须用 `geometry_msgs/PoseStamped`，在 object_slam 内把 Odometry 转成 PoseStamped，或加一个轻量桥接节点。

> 原版（HKUST ROS1）确实不发关键帧话题、需要补；**本 fork 已解决，勿再按旧文档重复添加**。旧版"补两个 publisher"的代码片段保留在 git 历史，不再适用。

## 5. D435i 配置

```bash
# 启动相机（D435i：双目红外 640x480 + IMU 200Hz，配置已固化）
ros2 launch realsense2_camera rs_launch.py

# 启动 VINS（在 /workspace 下运行）
ros2 run vins vins_node ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml

# 可视化（轨迹 + 双目红外 + 彩色）
rviz2 -d /workspace/ws_src/vins_fusion/config/vins_rviz_d435i.rviz
```

> 📄 节点入口：[vins/src/rosNodeTest.cpp](../ws_src/vins_fusion/vins/src/rosNodeTest.cpp)（可执行名 `vins_node`）；VINS 启动配置 [realsense_stereo_imu_config.yaml](../ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml)（`imu_topic`/`image0_topic`/`image1_topic`/`output_path` 分别见该文件 L8/L9/L10/L11）；rviz 配置 [vins_rviz_d435i.rviz](../ws_src/vins_fusion/config/vins_rviz_d435i.rviz)。`rs_launch.py` 在容器内被本项目覆盖版替换，见 [docker/realsense/rs_launch.py](../docker/realsense/rs_launch.py)。

**关闭节点**（进程 comm 被内核截断为 15 字符，`realsense` 用截断名）：

```bash
pkill -x vins_node
pkill -x realsense2_came     # 不是 realsense2_camera_node
pkill -x rviz2
```

> 注意区分两个目录：VINS 启动配置在 vins 仓库的 `ws_src/vins_fusion/config/realsense_d435i/`；而标定参数（`camera.yaml`/`imu.yaml`/`extrinsics.yaml`）是**本项目自己的** `config/d435i/`。标定值需要**手动填入 VINS 的 `realsense_stereo_imu_config.yaml`**。

配置项与项目 `config/d435i/` 三个标定文件的对应：

| VINS 配置项 | 来源 |
|---|---|
| `image_width/height`、`fx/fy/cx/cy`、畸变 | `camera.yaml` |
| `acc_n/acc_w/gyr_n/gyr_w` | `imu.yaml` |
| `body_T_cam0/body_T_cam1` | `extrinsics.yaml` |

## 6. 验证

```bash
# 看轨迹话题是否有数据（实测 ~15Hz）
ros2 topic hz /odometry
ros2 topic echo /odometry --once --field pose.pose

# 确认 TF（world→body）正常，rviz 才能显示轨迹
ros2 topic echo /tf --once

# rviz 可视化：轨迹 + 双目红外 + 彩色
rviz2 -d /workspace/ws_src/vins_fusion/config/vins_rviz_d435i.rviz
```

在 rviz 中手持 D435i 缓慢移动，观察：
1. 轨迹平滑、无跳变；
2. 点云与真实场景几何一致；
3. 静止时位置漂移 < 几厘米/分钟（粗略判断）。

## 7. 动态场景下的退化

VINS-Fusion 假设世界静态，动态物体（人走动）会污染点云、引入错误约束。两条应对路径：

- **路径 A（本项目的物体级路线）**：用 YOLO 分割把物体从背景分离，物体级模块只对静态背景做 VIO 约束（[`docs/05`](05_动态物体特征点剔除.md)、[`docs/07`](07_对偶二次曲面物体重建.md)）。
- **路径 B**：在 VINS 前端直接剔除落在动态 mask 内的特征点（[`docs/05`](05_动态物体特征点剔除.md) 的中间基线）。

## 8. 常见问题

| 现象 | 排查 |
|---|---|
| 收不到图像 | 确认 RealSense 话题名（`infra1/image_rect_raw`），改 VINS yaml 的 `image0_topic` |
| 轨迹发散 | 检查 `body_T_cam0` 外参是否正确、IMU 噪声是否填反 |
| 编译报 ceres/Eigen 缺失 | `libceres-dev libeigen3-dev` 已补进 Dockerfile；**旧镜像需手动** `apt-get install -y libceres-dev` |
| 编译报 `'CUDA' is not a member of 'ceres'` | 此 fork 的 [estimator.cpp](../ws_src/vins_fusion/vins/src/estimator/estimator.cpp#L1171-L1173) 用了 `ceres::CUDA`，而 apt 版 Ceres（2.0，无 CUDA）没有该枚举；已用 `#ifdef CERES_HAVE_CUDA` 保护，未定义时退化为 `DENSE_SCHUR` |
| 时间不同步导致丢帧 | 见 [`docs/04`](04_传感器时间同步与内外参标定.md) |
