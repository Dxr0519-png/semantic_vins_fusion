#!/usr/bin/env bash
set -e

# 挂载 USB 设备权限（D435i）
if [ -e /dev/bus/usb ]; then
  chmod -R a+rw /dev/bus/usb 2>/dev/null || true
fi

# 允许 root 访问 X（rviz）
if [ -n "$DISPLAY" ] && [ -e /root/.Xauthority ]; then
  xhost +local:root >/dev/null 2>&1 || true
fi

# 应用 realsense2_camera 的 launch 配置覆盖（从挂载的工作区取，容器重建也不丢）：
#   去 camera_namespace + IMU 默认全开（enable_gyro/accel=true, 400Hz, unite_imu_method=2）
if [ -f /workspace/docker/realsense/rs_launch.py ]; then
  cp /workspace/docker/realsense/rs_launch.py /opt/ros/humble/share/realsense2_camera/launch/rs_launch.py
fi

source /opt/ros/humble/setup.bash
[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash

exec "$@"
