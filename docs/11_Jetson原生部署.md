# 11 · Jetson 原生部署（板载直装）

## 1. 目标

在 **Jetson 板载系统**上直接安装并运行本项目，**不依赖 Docker**。
PC 上的 Docker 开发环境（docs/01）保持不变，两端共用同一份代码仓库与标定参数。

- **目标板**: reComputer Super J401 NX Bundle（Jetson Orin NX 16GB, Super 模式）
- **系统**: JetPack 6.2（L4T R36.4）= Ubuntu 22.04 + CUDA 12.6 + TensorRT 10.x
- **架构**: aarch64 · Python 3.10

> ⚠️ **关键前提**：ROS2 Humble 只支持 Ubuntu 22.04，即 **JetPack 6.x（L4T R36）**。
> JetPack 5（R35, Ubuntu 20.04）没有 Humble 的 apt 包，需降级到 Foxy，改动更大，**不建议**。

## 2. 板载直装 vs PC Docker 差异

| 组件 | PC Docker（docs/01） | 板载直装（本文档） |
|---|---|---|
| 基础镜像 | `osrf/ros:humble-desktop-full`（x86_64） | 无容器，Ubuntu 22.04 原生 |
| CUDA | 宿主驱动 + nvidia-container-toolkit | JetPack 自带（无需额外装） |
| torch | PyPI `cu130` wheel（x86_64） | NVIDIA JetPack 6 aarch64 wheel（**版本不同**） |
| tensorrt | pip 安装 | **JetPack 系统自带，禁止 pip** |
| onnxruntime-gpu | PyPI | 无 aarch64，用 Jetson 专用索引 |
| librealsense2 | `ros-humble-realsense2-camera`（amd64 deb） | **无 arm64 deb，源码编译** |
| g2o | Dockerfile 源码编译 | 同（源码编译 + `g2o_DIR`） |
| numpy | 锁 `<2`（Dockerfile 46-52 行） | 同样锁 `<2`（ABI 一致策略） |

代码本身（C++/Python/launch/msg）两端一致，差异全部在环境层。

## 3. 前置：刷写 JetPack 6.2 与基础验证

1. 用 NVIDIA SDK Manager / Seeed 官方镜像刷写 **JetPack 6.2**（出厂一般已预装）。
2. 首启验证：

```bash
uname -m                                   # 应输出 aarch64
python3 --version                          # 应为 3.10
sudo nvpmodel -m 0                         # 可选：最高性能档（功耗与散热自担）
python3 -c "import tensorrt; print(tensorrt.__version__)"   # 应为 10.x
```

## 4. 一键安装：scripts/setup_jetson.sh

脚本幂等，重复执行自动跳过已完成分节：

```bash
cd ~/semantic_vins_fusion        # 先同步项目代码到板子
./scripts/setup_jetson.sh        # 全量安装（会用到 sudo）
```

| 开关 | 作用 |
|---|---|
| `--skip-ros` | 跳过 apt + ROS2 Humble |
| `--skip-g2o` | 跳过 g2o 源码编译 |
| `--skip-realsense` | 跳过 librealsense2 + realsense-ros2 |
| `--skip-python` | 跳过 torch/ultralytics 等 Python 环境 |
| `--skip-colcon` | 跳过工作空间构建 |
| `--skip-trt` | 跳过 TRT engine 导出 |
| `--force` | 强制重跑对应分节（默认幂等跳过） |

> 无网络环境：先 `--skip-python --skip-trt` 装好系统部分，再手工拷贝 wheel 安装（见 5.4）。

## 5. 手工安装分解

> 出错排查用；一键跑通可跳过本节。

### 5.1 apt 基础 + ROS2 Humble（arm64）

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake pkg-config python3-pip python3-dev python3-opencv \
  libeigen3-dev libopencv-dev libboost-all-dev \
  libceres-dev libsuitesparse-dev libcholmod3 liblapack-dev libblas-dev

# ROS2 apt 源（Humble 提供 arm64 二进制）
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-humble-ros-base ros-humble-rviz2 ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-image-transport-plugins \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-message-filters \
  ros-humble-camera-info-manager ros-humble-diagnostic-updater \
  ros-humble-dynamic-reconfigure \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  python3-colcon-common-extensions
```

> **用 apt 的 `python3-opencv`，不要用 pip 的 `opencv-python`**：cv_bridge 的 C++ 扩展按 apt `libopencv-dev`（4.5.4）ABI 编译，pip 覆盖到 `/usr/local` 会在 import 时崩。

### 5.2 g2o 源码编译

Ubuntu 22.04 无 g2o apt 包，`object_slam` 的 `find_package(g2o REQUIRED)` 靠 `g2o_DIR` 定位：

```bash
git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o
cd /tmp/g2o && mkdir build && cd build
cmake .. -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install
echo 'export g2o_DIR=/usr/local/lib/cmake/g2o' >> ~/.bashrc
```

> `-DBUILD_WITH_MARCH_NATIVE=OFF` 必须保留，否则在 Jetson 上产生非法指令。

### 5.3 librealsense2 + realsense-ros2 源码 ★ D435i 高风险

arm64 **没有** `ros-humble-realsense2-camera` / `librealsense2` apt 包，只能源码：

```bash
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense
sudo cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
cd /tmp/librealsense && mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install && sudo ldconfig

# ROS2 包装层（与上方 SDK 版本匹配，master 分支即 ROS2）
git clone --depth 1 https://github.com/IntelRealSense/realsense-ros.git \
  ws_src/realsense-ros2
```

> ⚠️ **最高风险项**：`FORCE_RSUSB_BACKEND=true` 走 RSUSB 自定义 UVC 协议（避开 Jetson 内核 UVC 驱动缺陷），但 **D435i 的 IMU metadata / 硬件时间戳**在这条链路上已知易出问题。
> **务必先单独验证，再接 VINS**：
>
> ```bash
> rs-enumerate-devices                 # 相机枚举到、IMU 模块在
> ros2 launch realsense2_camera rs_launch.py
> ros2 topic hz /camera/imu            # IMU 频率正常（~400Hz）
> ros2 topic delay /camera/imu         # 时间戳抖动可控
> ```
>
> IMU 无数据时检查 `rs_launch.py` 的 `enable_accel/enable_gyro`，必要时 `unite_imu_method=2`。

### 5.4 Python aarch64 wheels

PyPI 没有 arm64 的 CUDA torch。策略与 Dockerfile 46-52 行一致：`--ignore-installed` 覆盖 apt 的 distutils 老包（sympy/mpmath/numpy），并**锁 numpy<2**：

```bash
sudo -H python3 -m pip install --upgrade pip setuptools wheel
sudo -H python3 -m pip install --ignore-installed sympy mpmath "numpy<2"

# torch / torchvision：NVIDIA 官方 JetPack 6 aarch64 索引（自动选 cp310 wheel）
sudo -H python3 -m pip install \
  https://developer.download.nvidia.com/compute/redist/jp/v62/pytorch/torch-2.7.0-cp310-cp310-linux_aarch64.whl \
  https://developer.download.nvidia.com/compute/redist/jp/v62/pytorch/torchvision-0.22.0a0+2a3b1296-cp310-cp310-linux_aarch64.whl
# 兜底索引：pypi.jetson-ai-lab.io/jp6/cu126

sudo -H python3 -m pip install ultralytics onnx
sudo -H python3 -m pip uninstall -y opencv-python opencv-contrib-python || true   # 让 apt python3-opencv 生效
sudo -H python3 -m pip install onnxruntime-gpu \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 --extra-index-url https://pypi.org/simple
sudo -H python3 -m pip install --ignore-installed "numpy<2"   # 兜底锁版
```

| 组件 | 来源 | 说明 |
|---|---|---|
| torch / torchvision | NVIDIA JetPack 6 索引（或 jetson-ai-lab） | **不用 cu130**（那是 x86_64） |
| tensorrt | **JetPack 系统自带** | 禁止 pip；缺 Python 绑定则 `apt install python3-libnvinfer` |
| onnxruntime-gpu | jetson-ai-lab 索引（jp6/cu126） | PyPI 无 aarch64 版 |
| ultralytics / onnx | PyPI | 常规安装 |

## 6. colcon 构建与运行

```bash
cd ~/semantic_vins_fusion          # workspace 根（含 ws_src/）
source /opt/ros/humble/setup.bash
export g2o_DIR=/usr/local/lib/cmake/g2o
colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

- **`--parallel-workers 4`**：Orin NX 16GB 并行 make 易 OOM，限制并发。
- `global_fusion` 已移除遗留的 rclpy 依赖；若它报错，可用 `--packages-select` 跳过（非主流程）。
- 运行前先建 VINS 输出目录（VINS 不自动创建）：

```bash
mkdir -p /tmp/vins_output /tmp/vins_output/pose_graph
```

启动顺序与 docs/10 一致（注意**从 workspace 根运行**，calib 相对配置目录解析）：

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 run vins vins_node ws_src/vins_fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml
ros2 launch yolo_seg_ros yolo_seg.launch.py
ros2 launch object_slam object_slam.launch.py
rviz2 -d ws_src/vins_fusion/config/vins_rviz_d435i.rviz   # 或 ros2 launch vins vins_rviz.launch.xml
```

## 7. TensorRT engine 板载导出

- `.pt` / `.onnx` 权重可在 PC 上准备，随仓库或 USB 拷到板子（`.pt` 已被 .gitignore 排除，不入库）。
- **`.engine` 与 GPU 架构 + TensorRT 版本绑定，必须在板子上就地导出**：

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run yolo_seg_ros export_trt --weights yolov8n-seg.pt --imgsz 640 --half
```

导出后把 `ws_src/yolo_seg_ros/config/yolo.yaml` 的 `model_path` 改为 `yolov8n-seg.engine`。

## 8. 性能建议

| 手段 | 说明 |
|---|---|
| `mask_scale: 0.25` | 降低 mask 下采样到 0.25，省推理与传输带宽（当前 0.5） |
| `publish_vis: false` | 关闭可视化叠加话题，省 CPU 与网络 |
| 关 rviz / 远程 rviz | 板载 headless 运行，rviz 放 PC 远程连 |
| `--parallel-workers 4` | colcon 构建控内存 |
| `nvpmodel` 功耗档 | Super 模式 MAXN 性能最高，注意散热 |

## 9. 常见问题（FAQ）

| 现象 | 原因 · 解决 |
|---|---|
| `/camera/imu` 无数据 | D435i metadata / RSUSB 链路问题，检查 `enable_accel/enable_gyro`、`unite_imu_method=2`（见 5.3） |
| `import tensorrt` 失败 | 系统 python 没绑 JetPack 的 TRT：`apt install python3-libnvinfer` |
| `torch.cuda.is_available()` 为 False | torch 装错成了 CPU/x86 版，重装 NVIDIA JetPack 6 aarch64 wheel（5.4） |
| cv_bridge import 崩 | pip 的 opencv-python 覆盖了 apt 版，`pip uninstall opencv-python` |
| `global_fusion` configure 报 rclpy 错 | 旧版本依赖，已清理；`--packages-select` 跳过非主流程包 |
| 离线无网 | `--skip-python --skip-trt` 先装系统部分，手工拷贝 wheel 文件安装 |
| `vins_node` 写不了 `/tmp/vins_output/vio.csv` | 忘建目录：`mkdir -p /tmp/vins_output` |

## 10. 下一步

- 端到端运行与分阶段验证 → [`docs/10`](10_运行与验证.md)
- VINS 接入与改动点 → [`docs/02`](02_VINS-Fusion_ROS2集成.md)
- YOLOv8-seg 集成与 TensorRT 加速 → [`docs/03`](03_YOLOv8-seg集成与TensorRT加速.md)
