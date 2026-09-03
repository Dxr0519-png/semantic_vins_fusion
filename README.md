# semantic_vins_fusion

基于 **VINS-Fusion**（双目+IMU 视觉惯性里程计）与 **YOLOv8-seg**（实例分割）的**物体级语义 SLAM** 项目。

在动态场景中，用语义分割把"物体"从背景中分离出来，用**对偶二次曲面（dual quadric / 椭球）**表达物体并联合优化，最终输出一条鲁棒的轨迹 + 一张带位姿、尺度、类别标签的语义物体地图。

> 说明：目录名沿用 `semantic_vins_fusion`（含拼写），文档正文统一写作 "semantic"。

---

## 概览

| 模块 | 作用 | 技术栈 |
| --- | --- | --- |
| `realsense-ros2` | D435i 驱动（双目 infra + 彩色 + IMU） | C++、ROS2（官方 4.x 源码版） |
| `vins_fusion` | 双目+IMU 视觉惯性里程计（位姿 + 稀疏点云） | C++、ROS2 移植版 |
| `yolo_seg_ros` | 2D 实例分割（bbox + mask + 类别） | Python、ultralytics、TensorRT |
| `object_slam` | 物体级数据关联、对偶二次曲面重建、联合优化 | C++、Eigen、g2o |
| `semantic_interfaces` | 跨节点自定义消息 | ROS2 msg（C++/Python 共用） |

**数据流**：

```text
                        ┌─ /camera/infra1,infra2 ──► vins_fusion ──► 位姿/点云/关键帧 ──┐
D435i ──► realsense2_camera ─┤                                                                          ├─► object_slam ──► 物体地图 + 优化轨迹
                        ├─ /camera/color ─────────► yolo_seg_ros ──► Detection2DArray ──┘
                        └─ /camera/imu ────────────► vins_fusion（IMU 预积分，~199Hz）
```

详细设计见 [`docs/00_项目概述与总体架构.md`](docs/00_项目概述与总体架构.md)。

---

## 部署方式

| 环境 | 方式 | 入口 |
| --- | --- | --- |
| PC 开发机（CUDA GPU） | Docker 容器（ROS2 Humble + CUDA + RealSense + ultralytics + g2o） | `scripts/build_docker.sh` + `scripts/run_container.sh` |
| Jetson 板载（reComputer Super J401，Orin NX 16GB） | 原生直装（JetPack 6.2，不用 Docker） | `scripts/setup_jetson.sh` |

---

## 快速开始（PC / Docker）

```bash
# 1. 构建并进入容器
./scripts/build_docker.sh
./scripts/run_container.sh

# 2. 容器内构建工作空间
cd /workspace
colcon build --symlink-install
source install/setup.bash

# 3. 依次启动（推荐按 docs/13 分阶段逐级验证）
ros2 launch vins rs_camera.launch.py      # D435i：双目 infra + 彩色 + IMU 全开
ros2 run vins vins_node ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml   # VIO（先启动相机）
rviz2 -d ws_src/vins_fusion/config/vins_rviz_d435i.rviz   # 可视化：VINS 轨迹 + 图像 + 物体椭球
ros2 launch yolo_seg_ros yolo_seg.launch.py         # 语义分割
ros2 launch object_slam object_slam.launch.py       # 物体级 SLAM
```

- 相机启动用项目封装的 `rs_camera.launch.py`，**不要**用 realsense 自带的 `rs_launch.py` —— 后者默认关闭 IMU 与 infra，VINS 收不到数据（封装说明见 `ws_src/vins_fusion/vins/launch/rs_camera.launch.py` 头注释）。
- 物体椭球在 rviz 中以半透明 Marker 显示（话题 `/object_slam/objects_markers`）。
- 关闭节点（realsense 进程名截断为 `realsense2_came`，`pkill -x` 按进程名匹配）：

  ```bash
  pkill -x vins_node && pkill -x realsense2_came && pkill -x rviz2
  ```

完整环境搭建 → [`docs/01`](docs/01_环境搭建(Docker+ROS2+CUDA+RealSense).md)；端到端运行与验证 → [`docs/10`](docs/10_运行与验证.md)。

## 快速开始（Jetson 板载）

```bash
./scripts/setup_jetson.sh        # 幂等：可重复执行，--skip-* 跳过对应分节
```

- 板载流程与 PC 相同：`rs_camera.launch.py → vins_node → rviz2 → yolo_seg → object_slam`（在板载 workspace 内）。
- D435i 的 IMU metadata / 硬件时间戳是**已知高风险**，务必先单独验证 `ros2 topic hz /camera/imu` 再接入 VINS。
- 板载环境配置分步执行见 [`docs/12`](docs/12_Jetson环境配置分步执行.md)，完整部署见 [`docs/11`](docs/11_Jetson原生部署.md)。

---

## 文档目录

| # | 文档 | 内容 |
| --- | --- | --- |
| 00 | [项目概述与总体架构](docs/00_项目概述与总体架构.md) | 总体设计、数据流、模块划分 |
| 01 | [环境搭建(Docker+ROS2+CUDA+RealSense)](docs/01_环境搭建(Docker+ROS2+CUDA+RealSense).md) | 容器与依赖 |
| 02 | [VINS-Fusion ROS2 集成](docs/02_VINS-Fusion_ROS2集成.md) | VIO 接入、话题、改动点 |
| 03 | [YOLOv8-seg 集成与 TensorRT 加速](docs/03_YOLOv8-seg集成与TensorRT加速.md) | 分割节点、模型导出 |
| 04 | [传感器时间同步与内外参标定](docs/04_传感器时间同步与内外参标定.md) | 同步与标定 |
| 05 | [动态物体特征点剔除](docs/05_动态物体特征点剔除.md) | 动态点过滤（基线） |
| 06 | [物体级数据关联与跟踪](docs/06_物体级数据关联与跟踪.md) | 跟踪、ID 维护 |
| 07 | [对偶二次曲面物体重建](docs/07_对偶二次曲面物体重建.md) | 椭球重建数学与实现 |
| 08 | [物体级联合优化(g2o 因子图)](docs/08_物体级联合优化(g2o因子图).md) | 联合 BA |
| 09 | [自定义消息与接口定义](docs/09_自定义消息与接口定义.md) | 消息协议 |
| 10 | [运行与验证](docs/10_运行与验证.md) | 端到端测试与评测 |
| 11 | [Jetson 原生部署](docs/11_Jetson原生部署.md) | 板载直装（JetPack 6.2） |
| 12 | [Jetson 环境配置分步执行](docs/12_Jetson环境配置分步执行.md) | 板载分步操作手册 |
| 13 | [分阶段启动与逐级验证](docs/13_分阶段启动与逐级验证.md) | 端到端逐级验证流程（推荐顺序） |
| 14 | [技术路线详解](docs/14_技术路线详解.md) | 算法路线与模块深挖 |
| 15 | [标定全流程速查](docs/15_标定全流程速查.md) | 标定快速参考 |
| 16 | [录屏验证与演示视频录制](docs/16_录屏验证与演示视频录制.md) | 录屏与演示视频制作 |

> 注：`docs/` 已在 `.gitignore` 中，为**本机文档**，不随仓库共享。

---

## 目录结构

```text
semantic_vins_fusion/
├── docs/                # 中文技术文档（本机，git 已忽略）
├── docker/              # 容器环境（Dockerfile、compose、realsense udev 规则）
├── ws_src/              # ROS2 工作空间 src
│   ├── realsense-ros2/  # D435i 官方驱动源码（realsense2_camera）
│   ├── semantic_interfaces/   # 自定义消息
│   ├── yolo_seg_ros/    # 分割节点（含 yolov8n-seg.pt/onnx/engine 模型）
│   ├── vins_fusion/     # VINS 移植 + rs_camera.launch.py 相机封装
│   └── object_slam/     # 物体级 SLAM（数据关联/重建/优化）
├── config/              # 标定参数 d435i/（camera、extrinsics、imu）+ 模型配置 yolo/
└── scripts/             # 构建/板载部署（setup_jetson）/标定（calib）/评测（eval）脚本
```

---

## 许可与出处

- 本项目为**研究与学习性质**。
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) 及社区 ROS2 移植版为 **GPLv3**，商用前请确认许可。
- 物体级重建部分参考 QuadricSLAM / SO-SLAM / OA-SLAM 思路。
- YOLOv8-seg 基于 [ultralytics](https://github.com/ultralytics/ultralytics)（AGPL-3.0）。
- realsense-ros2 为 [IntelRealSense 官方源码](https://github.com/IntelRealSense/realsense-ros)。
