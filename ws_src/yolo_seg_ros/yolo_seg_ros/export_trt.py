#!/usr/bin/env python3
"""
把 YOLOv8-seg 权重导出为 TensorRT engine，供 GPU 实时推理。

用法:
    ros2 run yolo_seg_ros export_trt --weights yolov8n-seg.pt --imgsz 640 --half
导出得到 yolov8n-seg.engine，写入 config/yolo.yaml 的 model_path 即可。
"""
import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description='导出 YOLOv8-seg 为 TensorRT')
    parser.add_argument('--weights', default='yolov8n-seg.pt')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--half', action='store_true', help='FP16 加速')
    args = parser.parse_args()

    model = YOLO(args.weights)
    path = model.export(format='engine', imgsz=args.imgsz, half=args.half)
    print(f'导出完成: {path}')
    print('请在 config/yolo.yaml 中把 model_path 改为该 .engine 文件路径')


if __name__ == '__main__':
    main()
