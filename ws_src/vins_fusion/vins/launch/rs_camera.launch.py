#!/usr/bin/env python3
# =====================================================================
# rs_camera.launch.py — 项目相机启动配置（封装 realsense2_camera）
#
# 用途：一键启动 D435i，输出本项目（VINS + YOLO-seg + object_slam）
#       需要的全部流。rs_launch.py 默认关闭 IMU 与 infra，必须显式开启。
#
# 参数说明（板载实测 2026-08-22）：
#   - enable_accel/enable_gyro: 开 IMU（默认 false）
#   - unite_imu_method=2: accel+gyro 融合为 /camera/imu（~199Hz）
#   - enable_infra1/2: VINS 左目/右目（默认 false）
#   - depth_module.infra_profile=640,480,30: 双目分辨率，匹配 VINS 配置
#     （realsense_stereo_imu_config.yaml 的 image_width/height=640x480）
#
# 用法：
#   ros2 launch vins rs_camera.launch.py
#   或  ros2 launch <本文件绝对路径>
# =====================================================================
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py',
        ]),
        launch_arguments={
            # 话题前缀：camera_namespace=/ + node 名 camera → /camera/...（单层）
            # （4.58.3 话题按 "~/stream/..." 私有话题构造，ns+node 决定前缀；
            #   默认 camera_namespace=camera 会是 /camera/camera/...，与 VINS/YOLO 配置不匹配）
            'camera_namespace': '/',
            # IMU（默认关闭，必须显式开启）
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'unite_imu_method': '2',      # 0-None, 1-copy, 2-linear_interpolation
            # 双目 infra（VINS 左目/右目，默认关闭）
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'depth_module.infra_profile': '640,480,30',
            # color 流固定 640x480x30（默认 1280x720，与 object_slam/YOLO 配置不匹配，2026-09-01 实测确认）
            'rgb_camera.color_profile': '640,480,30',
        }.items(),
    )
    return LaunchDescription([rs_launch_include])
