#!/usr/bin/env python3
"""
YOLOv8-seg 实例分割 ROS2 节点。

订阅:
    /camera/color/image_raw   (sensor_msgs/Image)
发布:
    /yolo/detections          (semantic_interfaces/Detection2DArray)
    /yolo/vis                 (sensor_msgs/Image, 可视化叠加结果, 可选)

模型: .pt 或导出的 .engine (TensorRT)，通过 config/yolo.yaml 的 model_path 指定。
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from semantic_interfaces.msg import Detection2D, Detection2DArray, InstanceMask
from ultralytics import YOLO


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')

        self.declare_parameter('model_path', 'yolov8n-seg.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('mask_scale', 0.5)          # mask 下采样比例
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('publish_vis', True)

        model_path = self.get_parameter('model_path').value
        self.conf = float(self.get_parameter('conf_thres').value)
        self.iou = float(self.get_parameter('iou_thres').value)
        self.mask_scale = float(self.get_parameter('mask_scale').value)
        image_topic = self.get_parameter('image_topic').value
        self.publish_vis = bool(self.get_parameter('publish_vis').value)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.det_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.vis_pub = self.create_publisher(Image, '/yolo/vis', 10) if self.publish_vis else None

        self.get_logger().info(f'YOLO 节点启动: model={model_path}, conf={self.conf}')

    def image_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(img, conf=self.conf, iou=self.iou,
                                     task='segment', verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header
        vis = img.copy() if self.publish_vis else None
        h, w = img.shape[:2]

        for r in results:
            if r.masks is None:
                continue
            masks = r.masks.data.cpu().numpy()            # (N,H,W) 0/1
            xyxy = r.boxes.xyxy.cpu().numpy()
            cls = r.boxes.cls.cpu().numpy().astype(int)
            confs = r.boxes.conf.cpu().numpy()
            names = r.names

            for i in range(masks.shape[0]):
                d = Detection2D()
                d.header = msg.header
                d.class_id = int(cls[i])
                d.class_name = names.get(int(cls[i]), str(int(cls[i])))
                d.score = float(confs[i])
                d.x_min = float(xyxy[i][0] / w)
                d.y_min = float(xyxy[i][1] / h)
                d.x_max = float(xyxy[i][2] / w)
                d.y_max = float(xyxy[i][3] / h)

                # mask 下采样（省带宽）
                m = masks[i].astype(np.uint8)
                if self.mask_scale != 1.0:
                    m = cv2.resize(m, None, fx=self.mask_scale, fy=self.mask_scale,
                                   interpolation=cv2.INTER_NEAREST)
                d.mask = InstanceMask()
                d.mask.width = int(m.shape[1])
                d.mask.height = int(m.shape[0])
                d.mask.data = m.flatten().tolist()

                det_array.detections.append(d)

                if self.publish_vis:
                    x1, y1, x2, y2 = map(int, xyxy[i])
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(vis, f'{d.class_name} {d.score:.2f}',
                                (x1, max(y1 - 5, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self.det_pub.publish(det_array)
        if self.publish_vis:
            self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
