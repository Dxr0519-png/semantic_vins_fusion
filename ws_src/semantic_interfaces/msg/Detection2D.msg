# 单个 2D 实例分割结果（来自 YOLOv8-seg）
std_msgs/Header header      # 对应原图帧；stamp 用于与 VINS 关键帧时间对齐
int32 class_id              # 类别 ID（默认 COCO 80 类）
string class_name           # 类别名（调试/可视化用）
float32 score               # 置信度 [0,1]

# 包围盒，归一化坐标 [0,1]（相对原图宽高）
float32 x_min
float32 y_min
float32 x_max
float32 y_max

InstanceMask mask           # 该实例的掩码
