# 标定脚本与流程指引

标定的完整步骤见 [`docs/04`](../../docs/04_传感器时间同步与内外参标定.md)，这里汇总可执行命令。

## 1. 查看出厂标定

```bash
rs-sensor-control
# Stereo Module -> 内参/畸变/基线
# Motion Module -> IMU 噪声、相机-IMU 外参
```

## 2. 复标相机（kalibr）

```bash
# 录标定板 bag
ros2 bag record -o calib_cam /camera/infra1/image_raw /camera/infra2/image_raw

# 标定双目（需要 AprilGrid target yaml）
kalibr_calibrate_cameras \
  --bag calib_cam \
  --topics /camera/infra1/image_raw /camera/infra2/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target april_6x6.yaml
```

## 3. 标定 IMU 噪声（imu_utils）

```bash
ros2 bag record -o imu_static /camera/imu   # 静置 2 小时
# 用 imu_utils 分析 Allan 方差 -> gyr_n/gyr_w/acc_n/acc_w
```

## 4. 标定相机-IMU 外参（kalibr）

```bash
kalibr_calibrate_imu_camera \
  --bag imu_cam.bag \
  --cam camchain.yaml --imu imu.yaml \
  --target april_6x6.yaml
```

## 5. 结果落地

把结果填入 [`config/d435i/`](../../config/d435i/) 的三个文件（camera.yaml / imu.yaml / extrinsics.yaml）。

> 标定板（AprilGrid）target yaml 文件放在本目录或 `config/` 下，与 kalibr 版本对应格式。
