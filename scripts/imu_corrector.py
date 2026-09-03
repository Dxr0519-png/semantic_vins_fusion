#!/usr/bin/env python3
"""IMU 校正节点：修正 D435i 加速度计/陀螺仪的 scale 与轴对齐误差。

数据来源：kalibr_calibrate_imu_camera --imu-models scale-misalignment（2026-09-01 实测）
  加速度计轴对齐误差 ~1.8°、scale 误差 ±0.7%（超出 MEMS 出厂典型值），
  导致 VINS 运动后静止漂移/外参估计发散。

模型（kalibr 约定）：measured = M @ true   →   true = inv(M) @ measured
  陀螺仪还含 g-灵敏度：gyro_meas = M_gyro @ gyro_true + A @ accel_true

用法：
  source /opt/ros/humble/setup.bash && source $WS/install/setup.bash
  python3 scripts/imu_corrector.py
  发布 /imu_corrected（best_effort），VINS imu_topic 指向它
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# kalibr 输出的 M（下三角 scale-misalignment），实测 2026-09-01
M_acc_inv = np.array([
    [ 1.00460162,  0.0,         0.0        ],
    [ 0.00235723,  0.99404896,  0.0        ],
    [-0.02586595, -0.03105946,  1.00455706 ],
])
M_gyro_inv = np.array([
    [ 1.00516549,  0.0,         0.0        ],
    [-0.00211877,  1.00682458,  0.0        ],
    [ 0.00303711,  0.00016615,  1.00207545 ],
])
A_gyro = np.array([  # 陀螺仪 g-灵敏度 [(rad/s)/(m/s^2)]，9/2 新标定
    [ 0.00018515,  0.00149111, -0.00030294],
    [-0.00059056,  0.00024169,  0.00033378],
    [ 0.00013339, -0.00044133, -0.00015332],
])


ACC_SPIKE = 3.0    # 相邻样本 accel 跳变阈值 m/s²（199Hz 下 5ms 间隔；物理运动 <1.5，超过视为坏样本）
GYR_SPIKE = 0.5    # 相邻样本 gyro 跳变阈值 rad/s


class ImuCorrector(Node):
    def __init__(self):
        super().__init__('imu_corrector')
        self.pub = self.create_publisher(Imu, '/imu_corrected', qos_profile_sensor_data)
        self.sub = self.create_subscription(Imu, '/camera/imu', self.cb, qos_profile_sensor_data)
        self.prev_a = None
        self.prev_g = None
        self.dropped = 0
        self.drop_streak = 0      # 2026-09-02：连续丢弃计数（启动坏样本毒化基准的兜底）
        self.get_logger().info('IMU 校正+滤波节点启动：/camera/imu → /imu_corrected')

    def cb(self, msg):
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        g = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # 尖峰滤波：相邻样本跳变超阈值视为坏样本，直接丢弃（VINS 按时间戳区间处理，可容忍缺样本）
        if self.prev_a is not None:
            if np.linalg.norm(a - self.prev_a) > ACC_SPIKE or np.linalg.norm(g - self.prev_g) > GYR_SPIKE:
                self.dropped += 1
                self.drop_streak += 1
                # 2026-09-02 实测修复：丢弃时不更新基准 → 启动首样本若为坏值
                # （流启动瞬间常见），基准被毒化后所有正常样本都被误判为尖峰、
                # 全部丢弃（真机实测 /imu_corrected 输出为空）。连续丢弃超
                # 1 秒（199Hz）视为基准失联 → 用当前样本重置基准恢复。
                if self.drop_streak >= 200:
                    self.prev_a = a
                    self.prev_g = g
                    self.drop_streak = 0
                if self.dropped % 50 == 1:
                    self.get_logger().warn(f'已丢弃尖峰样本 {self.dropped} 个'
                                           f'（连续 {self.drop_streak}）')
                return
        self.prev_a = a
        self.prev_g = g
        self.drop_streak = 0

        # scale/轴对齐校正（kalibr scale-misalignment 实测）
        a_c = M_acc_inv @ a
        g_c = M_gyro_inv @ (g - A_gyro @ a_c)
        msg.linear_acceleration.x = float(a_c[0])
        msg.linear_acceleration.y = float(a_c[1])
        msg.linear_acceleration.z = float(a_c[2])
        msg.angular_velocity.x = float(g_c[0])
        msg.angular_velocity.y = float(g_c[1])
        msg.angular_velocity.z = float(g_c[2])
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ImuCorrector())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
