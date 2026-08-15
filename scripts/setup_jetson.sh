#!/usr/bin/env bash
# =====================================================================
# Jetson 原生部署脚本（板载直装，不用 Docker）
#   目标板: reComputer Super J401 NX Bundle（Jetson Orin NX 16GB）
#   JetPack 6.2 (L4T R36.4) · Ubuntu 22.04 · CUDA 12.6 · TensorRT 10.x
#   aarch64 · Python 3.10
#
#   PC 开发仍用 docker/（不受影响），本脚本只负责板载环境。
#   幂等：重复执行会自动跳过已完成步骤；加 --force 强制重跑对应分节。
#
# 用法:
#   ./scripts/setup_jetson.sh [--skip-ros] [--skip-g2o] [--skip-realsense]
#                             [--skip-python] [--skip-colcon] [--skip-trt]
#                             [--force]
#
# 说明:
#   - 网络受限时先 --skip-python --skip-trt，手工拷 wheel 后单独跑 python 段。
#   - D435i 的 IMU metadata / 硬件时间戳是已知高风险，务必先单独验证
#     rs-enumerate-devices → ros2 topic hz /camera/imu 再接 VINS。
#   - 全部步骤见 docs/11_Jetson原生部署.md。
# =====================================================================
set -euo pipefail

# ---------- 0. 命令行开关 ----------
SKIP_ROS=0; SKIP_G2O=0; SKIP_REALSENSE=0; SKIP_PYTHON=0; SKIP_COLCON=0; SKIP_TRT=0; FORCE=0
for a in "$@"; do
  case "$a" in
    --skip-ros)       SKIP_ROS=1 ;;
    --skip-g2o)       SKIP_G2O=1 ;;
    --skip-realsense) SKIP_REALSENSE=1 ;;
    --skip-python)    SKIP_PYTHON=1 ;;
    --skip-colcon)    SKIP_COLCON=1 ;;
    --skip-trt)       SKIP_TRT=1 ;;
    --force)          FORCE=1 ;;
    --help|-h) echo "用法: $0 [--skip-ros] [--skip-g2o] [--skip-realsense] [--skip-python] [--skip-colcon] [--skip-trt] [--force]"; exit 0 ;;
    *) echo "未知参数: $a"; echo "用法见脚本头部注释（或 --help）"; exit 1 ;;
  esac
done

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"      # 本项目根目录（即 workspace 根）
export DEBIAN_FRONTEND=noninteractive

say(){ echo -e "\n==> $*"; }
warn(){ echo "    [!] $*"; }

# ---------- 1. 环境检查 ----------
say "1. 环境检查"
[ "$(uname -m)" = "aarch64" ] || { echo "错误: 仅支持 arm64（Jetson）"; exit 1; }
grep -qi "R36" /etc/nv_tegra_release 2>/dev/null || \
  warn "未识别 L4T R36（JetPack 6.x），请核对系统版本（JetPack 5 是 R35，无 ROS2 Humble）"
PY_MINOR="$(python3 -c 'import sys; print(sys.version_info.minor)' 2>/dev/null || echo '?')"
[ "$PY_MINOR" = "10" ] || warn "系统 python 应为 3.10（JetPack 6 标准），当前为 3.$PY_MINOR"
sudo -v || { echo "需要 sudo 权限"; exit 1; }
echo "    目标: $(uname -m) · 工作区: ${REPO_ROOT}"

# ---------- 2. apt 基础依赖 ----------
if [ $SKIP_ROS -eq 0 ]; then
  say "2. apt 基础依赖"
  sudo apt-get update
  # 注意: 用 apt 的 python3-opencv 而不用 pip 的 opencv-python，
  # 避免与 ros-humble-cv-bridge 链接的 apt libopencv-dev (4.5.4) 产生 ABI 冲突。
  sudo apt-get install -y --no-install-recommends \
    git curl wget vim tmux htop unzip \
    build-essential cmake pkg-config \
    python3-pip python3-dev python3-opencv \
    libeigen3-dev libopencv-dev libboost-all-dev \
    libceres-dev libsuitesparse-dev libcholmod3 liblapack-dev libblas-dev \
    libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    mesa-utils
fi

# ---------- 3. ROS2 Humble (arm64) + ros-humble-* 依赖 ----------
if [ $SKIP_ROS -eq 0 ]; then
  say "3. ROS2 Humble + 项目运行依赖"
  # 3.1 官方 ROS2 apt 源（Humble 提供 arm64 二进制，JetPack 6 / Ubuntu 22.04 直接支持）
  if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi
  if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
  fi

  # 3.2 本项目各包运行依赖（不装 desktop-full，省空间）
  sudo apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rviz2 ros-humble-cv-bridge \
    ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-tf2 ros-humble-tf2-ros \
    ros-humble-camera-info-manager ros-humble-diagnostic-updater \
    ros-humble-dynamic-reconfigure ros-humble-message-filters \
    ros-humble-rosgraph-msgs \
    ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
    python3-colcon-common-extensions

  # 3.3 bashrc 注入
  grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc 2>/dev/null || \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
fi

# ---------- 4. g2o 源码编译（无 apt / rosdep 包） ----------
if [ $SKIP_G2O -eq 0 ]; then
  say "4. g2o 源码编译"
  # object_slam 的 CMakeLists 是 find_package(g2o REQUIRED)，靠 g2o_DIR 环境变量定位
  if [ $FORCE -eq 1 ] || [ ! -f /usr/local/lib/cmake/g2o/g2oConfig.cmake ]; then
    rm -rf /tmp/g2o
    git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o
    cd /tmp/g2o && mkdir -p build && cd build
    # -DBUILD_WITH_MARCH_NATIVE=OFF: 必须关，否则在 Jetson 上产生非法指令
    cmake .. -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release
    make -j"$(nproc)"
    sudo make install
    sudo ldconfig
    rm -rf /tmp/g2o
  else
    echo "    g2o 已安装，跳过"
  fi
  grep -q "export g2o_DIR" ~/.bashrc 2>/dev/null || \
    echo 'export g2o_DIR=/usr/local/lib/cmake/g2o' >> ~/.bashrc
fi

# ---------- 5. librealsense2 源码（arm64 无 apt 包） ----------
if [ $SKIP_REALSENSE -eq 0 ]; then
  say "5. librealsense2 源码（D435i 驱动）"
  if [ $FORCE -eq 1 ] || ! pkg-config --exists realsense2 2>/dev/null; then
    rm -rf /tmp/librealsense
    git clone --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense
    # udev 规则（免 sudo 访问 D435i）
    sudo cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd /tmp/librealsense && mkdir -p build && cd build
    # FORCE_RSUSB_BACKEND: 走 RSUSB 自定义 UVC 协议，避开 Jetson 内核 UVC 驱动缺陷
    cmake .. -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=Release
    make -j"$(nproc)"
    sudo make install
    sudo ldconfig
    rm -rf /tmp/librealsense
  else
    echo "    librealsense2 已安装，跳过"
  fi

  # 6. realsense-ros2 源码（与 librealsense2 一起，供 colcon 构建）
  say "6. realsense-ros2 源码"
  if [ ! -d "${REPO_ROOT}/ws_src/realsense-ros2" ]; then
    # master 分支即 ROS2（4.x 对 ROS2）；建议 tag 与上方 librealsense2 版本匹配
    git clone --depth 1 https://github.com/IntelRealSense/realsense-ros.git \
      "${REPO_ROOT}/ws_src/realsense-ros2"
  else
    echo "    realsense-ros2 已存在，跳过"
  fi
fi

# ---------- 7. Python aarch64 wheels（系统 python3.10，勿建 venv） ----------
# 与 docker/Dockerfile 46-52 行同策略：apt 的 distutils 老包（sympy/mpmath/numpy）
# 无法被 pip 卸载，用 --ignore-installed 强制覆盖到 /usr/local（优先级高于 /usr/lib）。
# numpy 锁 <2：apt cv2 与 ROS cv_bridge 按 numpy 1.x ABI 编译。
if [ $SKIP_PYTHON -eq 0 ]; then
  say "7. Python aarch64 wheels"
  sudo -H python3 -m pip install --upgrade pip setuptools wheel
  sudo -H python3 -m pip install --ignore-installed sympy mpmath "numpy<2"

  # 7.1 torch/torchvision：PyPI 无 arm64 CUDA 版，用 NVIDIA 官方 JetPack 6 索引
  TORCH_BASE="https://developer.download.nvidia.com/compute/redist/jp/v62/pytorch"
  TORCH_WHEEL="$(curl -fsSL "${TORCH_BASE}/" 2>/dev/null \
    | grep -oE 'torch-2\.[0-9.]+[^"<>]*cp310-cp310-linux_aarch64\.whl' | sort -V | tail -1 || true)"
  TORCHV_WHEEL="$(curl -fsSL "${TORCH_BASE}/" 2>/dev/null \
    | grep -oE 'torchvision-[0-9][^"<>]*cp310-cp310-linux_aarch64\.whl' | sort -V | tail -1 || true)"
  if [ -n "$TORCH_WHEEL" ] && [ -n "$TORCHV_WHEEL" ]; then
    echo "    NVIDIA 索引找到 ${TORCH_WHEEL}"
    sudo -H python3 -m pip install "${TORCH_BASE}/${TORCH_WHEEL}" "${TORCH_BASE}/${TORCHV_WHEEL}"
  else
    # 兜底：Jetson AI Lab 索引同样提供 aarch64 + CUDA 的 torch/torchvision
    warn "NVIDIA 索引未匹配到 wheel，改用 jetson-ai-lab 索引（jp6/cu126）"
    sudo -H python3 -m pip install torch torchvision \
      --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
      --extra-index-url https://pypi.org/simple
  fi

  # 7.2 ultralytics + onnx（PyPI；torch 已装，pip 不会覆盖）
  sudo -H python3 -m pip install ultralytics onnx

  # 7.3 卸载 pip 的 opencv，让 apt python3-opencv 生效（与 cv_bridge ABI 一致）
  sudo -H python3 -m pip uninstall -y opencv-python opencv-contrib-python || true

  # 7.4 onnxruntime-gpu：PyPI 无 aarch64，用 Jetson 专用 wheel 索引（CUDA 12.6）
  sudo -H python3 -m pip install onnxruntime-gpu \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
    --extra-index-url https://pypi.org/simple || \
  warn "onnxruntime-gpu 安装失败，可手工安装离线 wheel（见 docs/11 §5.4）"

  # 7.5 兜底锁 numpy<2（torch/ort 可能拉回 numpy 2.x）
  sudo -H python3 -m pip install --ignore-installed "numpy<2"

  # 7.6 tensorrt：JetPack 系统自带，禁止 pip！验证即可
  python3 -c "import tensorrt; print('    tensorrt', tensorrt.__version__)" \
    || { warn "tensorrt 不可导入，尝试安装 JetPack 的 python 绑定"; \
         sudo apt-get install -y python3-libnvinfer || warn "python3-libnvinfer 安装失败"; }
fi

# ---------- 8. colcon 构建（含 realsense-ros2 与项目包） ----------
if [ $SKIP_COLCON -eq 0 ]; then
  say "8. colcon 构建"
  source /opt/ros/humble/setup.bash
  export g2o_DIR=/usr/local/lib/cmake/g2o
  # --parallel-workers 4: 控内存（Orin NX 16GB，并行 make 可能 OOM）
  cd "${REPO_ROOT}"
  colcon build --symlink-install --parallel-workers 4 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  grep -q "source ${REPO_ROOT}/install/setup.bash" ~/.bashrc 2>/dev/null || \
    echo "source ${REPO_ROOT}/install/setup.bash 2>/dev/null" >> ~/.bashrc
  echo "    colcon 完成。若 global_fusion 报错可用 --packages-select 跳过（见 docs/11 §6）"
fi

# ---------- 9. (可选) TensorRT engine 板载导出 ----------
if [ $SKIP_TRT -eq 0 ] && [ -f "${REPO_ROOT}/ws_src/yolo_seg_ros/yolov8n-seg.pt" ]; then
  say "9. TensorRT engine 板载导出"
  source /opt/ros/humble/setup.bash
  source "${REPO_ROOT}/install/setup.bash"
  cd "${REPO_ROOT}"
  python3 -c "import torch; assert torch.cuda.is_available(), 'GPU 不可用'" || { echo "    torch GPU 不可用，跳过导出"; exit 0; }
  ros2 run yolo_seg_ros export_trt --weights yolov8n-seg.pt --imgsz 640 --half
  echo "    导出完成，把 ws_src/yolo_seg_ros/config/yolo.yaml 的 model_path 改为 yolov8n-seg.engine"
else
  [ $SKIP_TRT -eq 1 ] || warn "未找到 yolov8n-seg.pt（离线部署请先预置模型），跳过 TRT 导出"
fi

# ---------- 10. 输出目录预建（VINS 不会自动创建） ----------
mkdir -p /tmp/vins_output /tmp/vins_output/pose_graph

echo -e "\n全部完成。重启终端后按 docs/11 §6 运行节点。"
