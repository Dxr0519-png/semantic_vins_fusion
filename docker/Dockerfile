# syntax=docker/dockerfile:1.4
# =====================================================================
# semantic_vins_fusion 开发容器
#   ROS2 Humble + CUDA(宿主驱动) + RealSense + ultralytics + TensorRT + g2o
#
# 构建:  docker compose build   （或 ./scripts/build_docker.sh）
# GPU 通过 nvidia-container-toolkit 挂载宿主驱动，镜像内无需安装 CUDA toolkit
# =====================================================================

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,video

# ---------- 1. 基础工具 ----------
# --mount=type=cache 让 apt 的下载与索引跨构建复用(BuildKit 缓存,不进镜像)
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && apt-get install -y --no-install-recommends \
        git curl wget vim tmux htop \
        build-essential cmake pkg-config \
        python3-pip python3-dev \
        libeigen3-dev libopencv-dev libboost-all-dev \
        libceres-dev libsuitesparse-dev libcholmod3 liblapack-dev libblas-dev \
        mesa-utils

# ---------- 2. ROS2 附加包：RealSense 驱动 / 图像传输 / 点云 ----------
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt/lists \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-realsense2-camera \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-pcl-ros \
        ros-humble-tf2-ros \
        ros-humble-rviz2

# ---------- 3. Python：torch(CUDA) + ultralytics + onnx + tensorrt ----------
# 按宿主 CUDA 版本选择 wheel 索引（RTX 5060 / Blackwell 需 cu126+，这里用 cu130）
# 注：已去掉 pycuda（项目代码未使用；且其无预编译 wheel，安装需 CUDA toolkit 头文件）
# --mount=type=cache 让 pip 下载缓存跨构建复用（BuildKit 缓存，不进镜像）
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --upgrade pip setuptools wheel
# 基础镜像 apt 装了 distutils 老包（sympy 1.9 / mpmath / numpy 1.21），pip 无法卸载它们，
# torch/ultralytics 的新依赖会触发 "Cannot uninstall ...distutils" 报错。
# 不能用 apt remove（会级联删掉 ros-humble-desktop-full / cv-bridge 等），
# 改用 --ignore-installed 强制覆盖为新版；/usr/local 排在 /usr/lib 前，导入时新版生效。
# numpy 锁 <2，避免与 apt cv2 / ROS 扩展的 ABI 不兼容。
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install --ignore-installed sympy mpmath "numpy<2"
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install \
        torch torchvision --index-url https://download.pytorch.org/whl/cu130
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 install \
        ultralytics onnx onnxruntime-gpu tensorrt \
        numpy opencv-python

# ---------- 4. g2o（物体级联合优化因子图库，源码编译） ----------
RUN git clone --depth 1 https://github.com/RainerKuemmerle/g2o.git /tmp/g2o && \
    cd /tmp/g2o && mkdir build && cd build && \
    cmake .. -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_EXAMPLES=OFF && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/g2o
ENV g2o_DIR=/usr/local/lib/cmake/g2o

# ---------- 5. 工作空间 ----------
WORKDIR /workspace

# ---------- 6. 环境与入口 ----------
# 容器内构建前需 source ROS 环境
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspace/install/setup.bash 2>/dev/null' >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
# 覆盖 realsense2_camera 的 launch 默认值：
#   去掉 camera_namespace（话题 /camera/... 而非 /camera/camera/...）
#   IMU 全开：enable_accel/enable_gyro=true, gyro_fps/accel_fps=400, unite_imu_method=2(合成 /camera/imu)
# 来源：docker/realsense/rs_launch.py（改自 apt 包自带版本）
COPY realsense/rs_launch.py /opt/ros/humble/share/realsense2_camera/launch/rs_launch.py
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
