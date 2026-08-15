#!/usr/bin/env bash
# =====================================================================
# 备用：手动安装 librealsense2 SDK（当 apt 的 ros-humble-realsense2-camera
# 已含 SDK 时无需运行此脚本；仅用于需要从源码装最新 SDK 的场景）
# =====================================================================
set -e

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    git cmake libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# udev 规则（免 sudo 访问相机）
cd /tmp
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=false -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
sudo make install

echo "librealsense2 安装完成，用 rs-enumerate-devices 验证"
