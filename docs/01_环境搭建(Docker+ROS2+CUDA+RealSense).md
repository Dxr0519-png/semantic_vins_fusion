# 01 · 环境搭建（Docker + ROS2 + CUDA + RealSense）

## 1. 目标

在宿主机上搭建一个可复现的开发容器，包含：

| 组件 | 版本/来源 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble（`osrf/ros:humble-desktop-full`） |
| CUDA | 宿主驱动（通过 nvidia-container-toolkit 挂载） |
| RealSense | `librealsense2` + `ros-humble-realsense2-camera` |
| Python 推理 | `torch`(CUDA) + `ultralytics` + `onnx` + `tensorrt` |
| 优化库 | `g2o`（源码编译） |

## 2. 宿主机前置条件

```bash
# 1) 安装 Docker
curl -fsSL https://get.docker.com | sh

# 2) 安装 nvidia-container-toolkit（关键，缺了容器内看不到 GPU）
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker

# 3) 验证宿主 GPU 驱动
nvidia-smi
```

> 若 `nvidia-smi` 报错，先装宿主 NVIDIA 驱动；容器的 CUDA 版本取决于宿主驱动，因此 `Dockerfile` 里 torch 的 wheel 索引（cu121/cu118）需与 `nvidia-smi` 顶部显示的 CUDA 版本匹配。

## 3. 构建镜像

```bash
cd docker
docker compose build
# 或
docker build -t semantic_vins_fusion:humble .
```

> 📄 镜像定义见 [docker/Dockerfile](../docker/Dockerfile)（g2o 源码编译 L62-66、`g2o_DIR` L67、numpy 锁 `<2` L46-52、torch 安装 L54-55）与 [docker/docker-compose.yml](../docker/docker-compose.yml)；等价脚本 [scripts/build_docker.sh](../scripts/build_docker.sh)。

首次构建需下载 ROS2 基础镜像与各依赖，约 10~20 分钟。

## 4. 启动容器

```bash
# 允许容器访问 X（rviz 可视化）
xhost +local:root

cd docker
docker compose up -d
docker exec -it semantic_vins_fusion bash
# 或使用脚本
../scripts/run_container.sh
```

> 📄 对应 [docker/docker-compose.yml](../docker/docker-compose.yml)（`runtime: nvidia` 等 GPU 挂载）+ [docker/entrypoint.sh](../docker/entrypoint.sh)（容器入口）；等价脚本 [scripts/run_container.sh](../scripts/run_container.sh)。

## 5. 容器内验证

```bash
# 5.1 GPU 可用性（应显示宿主机 GPU）
nvidia-smi

# 5.2 torch 能否用 CUDA
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

# 5.3 TensorRT 可导入
python3 -c "import tensorrt; print(tensorrt.__version__)"

# 5.4 RealSense 相机可见
rs-enumerate-devices
# 注意：该工具（librealsense2-utils）默认不在容器内；宿主机已装则直接在宿主机验证，
#       容器内需要：apt-get install -y librealsense2-utils

# 5.5 ROS2 环境
echo $ROS_DISTRO          # humble
ros2 --help
```

> 📄 容器内相关依赖（torch/ultralytics/tensorrt、RealSense、ROS2 包）的安装即 [docker/Dockerfile](../docker/Dockerfile) 的第 1~3 段；RealSense 话题前缀等 launch 改动见 [docker/realsense/rs_launch.py](../docker/realsense/rs_launch.py)。

## 6. 构建 ROS2 工作空间

容器内 `/workspace` 已挂载为本项目根目录：

```bash
cd /workspace
# 先获取 vins_fusion 社区移植（见 docs/02）
colcon build --symlink-install
source install/setup.bash
```

## 7. 常见问题（FAQ）

| 现象 | 原因 / 解决 |
|---|---|
| 容器内 `nvidia-smi` 无输出 | 未装/未重启 nvidia-container-toolkit，或 compose 缺 `runtime: nvidia` |
| `torch.cuda.is_available()` 为 False | wheel 索引 cu 版本与宿主驱动不匹配，重装匹配版本 |
| `rs-enumerate-devices` 找不到设备 | ① 宿主机 `lsusb` 确认有 `8086:0b3a`；② **相机在容器启动后才插入时，需重建容器** `docker compose up -d --force-recreate`（`/dev/bus/usb` 是容器创建时的快照，热插拔不会自动更新）；③ 容器内需装 `librealsense2-utils`；④ udev 权限/`devices: /dev/bus/usb` |
| rviz 打不开窗口 | X11 未挂载；确认 `xhost +local:root` 与 `DISPLAY` |
| 编译 g2o 报 Eigen 错误 | 已用 `BUILD_WITH_MARCH_NATIVE=OFF` 规避；确认 `libeigen3-dev` 已装 |
| `colcon build` 找不到消息包依赖 | 确认 `source /opt/ros/humble/setup.bash`，且消息包先于依赖包构建（colcon 自动处理拓扑序） |

## 8. 下一步

- 获取并配置 VINS-Fusion 移植 → [`docs/02`](02_VINS-Fusion_ROS2集成.md)
- 打通 YOLO 分割节点 → [`docs/03`](03_YOLOv8-seg集成与TensorRT加速.md)
