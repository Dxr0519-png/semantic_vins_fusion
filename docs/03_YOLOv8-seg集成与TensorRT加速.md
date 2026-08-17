# 03 · YOLOv8-seg 集成与 TensorRT 加速

## 1. 目标

实现一个 ROS2 节点，订阅 D435i 左目彩色图，用 **YOLOv8-seg** 做实例分割，输出带 bbox + mask + 类别的 `Detection2DArray`，供物体级模块使用；用 **TensorRT** 把推理加速到实时。

## 2. 原理要点

- **实例分割**：区别于目标检测（只给 bbox），YOLOv8-seg 额外输出每个实例的逐像素掩码，用于"物体 mask 内 3D 点关联"（[`docs/07`](07_对偶二次曲面物体重建.md) 的关键输入）。
- **TensorRT**：把 PyTorch 模型量化为 FP16 的 engine，在 NVIDIA GPU 上推理，典型可提升 2~4 倍吞吐。
- **模型选择**：`yolov8n-seg`（nano，快）→ `yolov8s-seg`（平衡）→ `yolov8m-seg`（准）。先用 nano 跑通，再按精度需求换大。

## 3. 文件结构

```
ws_src/yolo_seg_ros/
├── package.xml
├── setup.py / setup.cfg
├── resource/yolo_seg_ros
├── yolo_seg_ros/
│   ├── __init__.py
│   ├── yolo_seg_node.py     # 主节点
│   └── export_trt.py        # TensorRT 导出
├── launch/yolo_seg.launch.py
└── config/yolo.yaml
```

## 4. 节点实现解析（yolo_seg_node.py）

### 4.1 参数（config/yolo.yaml）

| 参数 | 含义 | 默认 |
|---|---|---|
| `model_path` | `.pt` 或 `.engine` | `yolov8n-seg.pt` |
| `conf_thres` | 置信度阈值 | 0.5 |
| `iou_thres` | NMS IoU 阈值 | 0.45 |
| `mask_scale` | mask 下采样比例（省带宽） | 0.5 |
| `image_topic` | 订阅图像话题 | `/camera/color/image_raw` |
| `publish_vis` | 是否发可视化图 | true |

### 4.2 关键代码

```python
# 订阅（RealSense 常用 Best Effort QoS）
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.BEST_EFFORT)
self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos)

# 推理
results = self.model.predict(img, conf=self.conf, iou=self.iou,
                             task='segment', verbose=False)

# 遍历实例，归一化 bbox + 下采样 mask
d.x_min = float(xyxy[i][0] / w)   # 归一化，与分辨率无关
d.mask.data = m.flatten().tolist()
```

> 📄 **源码出处**（文档为节选示意，非逐行一致）：[yolo_seg_node.py](../ws_src/yolo_seg_ros/yolo_seg_ros/yolo_seg_node.py) —— QoS 定义 L45-46、订阅 L48、推理 L56-57、归一化 bbox L79-82、mask 下采样+展平 L85-92。

**设计点**：bbox 用**归一化坐标**，object_slam 反投影时乘以原图尺寸即可，避免分辨率耦合。

## 5. TensorRT 导出

> **第一次先编译工作空间**。`ros2 run` 只能识别已安装的包，新代码必须先 `colcon build`：

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select semantic_interfaces yolo_seg_ros
```

> **每次新开终端都要 source**，否则 `ros2 run` 报 `Package 'yolo_seg_ros' not found`：

```bash
source /workspace/install/setup.bash
```

然后导出 engine（首次会自动下载权重）：

```bash
# 容器内（首次会自动下载权重）
ros2 run yolo_seg_ros export_trt --weights yolov8n-seg.pt --imgsz 640 --half
# 生成 yolov8n-seg.engine
```

> 📄 对应入口脚本：[yolo_seg_ros/yolo_seg_ros/export_trt.py](../ws_src/yolo_seg_ros/yolo_seg_ros/export_trt.py)；`model_path` 等参数配置见 [config/yolo.yaml](../ws_src/yolo_seg_ros/config/yolo.yaml)。

然后把 `config/yolo.yaml` 的 `model_path` 改为 engine 路径，重启节点即用 TensorRT 推理。

> 导出前确认 `torch.cuda.is_available()` 为 True；engine 与 GPU 型号/CUDA/TensorRT 版本绑定，换机器需重新导出。

## 6. 运行与验证

```bash
# 终端 1：启动 D435i（若已装 realsense2_camera）
ros2 launch realsense2_camera rs_launch.py

# 终端 2：启动 YOLO
ros2 launch yolo_seg_ros yolo_seg.launch.py

# 终端 3：查看输出
ros2 topic echo /yolo/detections --no-arr   # 看 detections 数量
ros2 topic echo /yolo/detections --once | grep -E "class_name|score|class_id"  # 看类型/置信度/bbox（一次）
ros2 topic echo /yolo/detections | grep class_name        # 实时看类型
ros2 topic hz /yolo/detections              # 看帧率（应 ≥15Hz）

# 可视化：rviz2 订阅 /yolo/vis 或 /camera/color/image_raw
rviz2
```

> 注:`--no-arr` 会把 `detections` 序列也折叠（只显示数量、看不到类型）。要看检测类型用 `grep` 过滤即可——`mask.data` 大数组同样被 grep 滤掉，不会刷屏。

## 7. 常见问题

| 现象 | 排查 |
|---|---|
| `Package 'yolo_seg_ros' not found` | 还没编译 / 没 source：先 `colcon build --packages-select semantic_interfaces yolo_seg_ros`，再 `source /workspace/install/setup.bash` |
| 收不到图像 | 确认 RealSense 话题名（`ros2 topic list \| grep color`），改 `image_topic` |
| 帧率低（<10Hz） | 用 `.engine` 而非 `.pt`；或换 `yolov8n-seg`；降低输入分辨率 |
| `import tensorrt` 失败 | Dockerfile 未装 tensorrt，或宿主驱动版本不匹配 |
| mask 过大占带宽 | 调小 `mask_scale`（0.25） |

## 8. 下一步

- 把检测结果喂给 object_slam 做数据关联 → [`docs/06`](06_物体级数据关联与跟踪.md)
- 用 mask 剔除动态特征点（可选中间基线）→ [`docs/05`](05_动态物体特征点剔除.md)
