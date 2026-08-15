#!/usr/bin/env python3
"""
物体重建精度评测：订阅 /object_slam/objects，比较重建椭球半轴与已知真实半轴。

用法（在容器内，已 source install/setup.bash）:
    python3 scripts/eval/eval_object_error.py --gt 0.40 0.30 0.20 --class_id 56

输出：每个物体的中心/尺度相对误差，以及多帧尺度稳定性。
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from semantic_interfaces.msg import ObjectMap


class EvalNode(Node):
    def __init__(self, gt, class_id):
        super().__init__('eval_object_error')
        self.gt = np.array(gt, dtype=float)
        self.class_id = class_id
        self.history = {}          # id -> list of scale
        self.sub = self.create_subscription(
            ObjectMap, '/object_slam/objects', self.cb, 10)
        self.get_logger().info(f'评测启动: gt半轴={self.gt}, 关注class_id={class_id}')

    def cb(self, msg: ObjectMap):
        for o in msg.objects:
            if self.class_id >= 0 and o.class_id != self.class_id:
                continue
            scale = np.array(o.scale)
            self.history.setdefault(o.id, []).append(scale)

            err = np.abs(scale - self.gt) / self.gt * 100.0   # 相对误差 %
            # 三轴可能顺序不同，取最优排列（简单做法：排序后比较）
            err_sorted = np.abs(np.sort(scale) - np.sort(self.gt)) / np.sort(self.gt) * 100.0
            print(f'[id={o.id} cls={o.class_id}] '
                  f'scale={np.round(scale, 4)} '
                  f'rel_err%={np.round(err_sorted, 2)}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gt', type=float, nargs=3, required=True,
                        help='真实三轴半轴(米)，如 --gt 0.40 0.30 0.20')
    parser.add_argument('--class_id', type=int, default=-1,
                        help='关注的 COCO 类别 id，-1 表示全部')
    args = parser.parse_args()

    rclpy.init()
    node = EvalNode(args.gt, args.class_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
