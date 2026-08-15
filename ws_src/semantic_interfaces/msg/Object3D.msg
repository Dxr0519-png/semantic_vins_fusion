# 一个 3D 物体（对偶二次曲面 / 椭球表示）
std_msgs/Header header
int32 id                    # track_id：跨帧稳定的物体标识
int32 class_id
string class_name
float32 confidence

float64[16] dual_quadric    # 4x4 对称矩阵 Q*（对偶二次曲面，行主序），世界系下
geometry_msgs/Point center  # 椭球中心（世界系）
float64[3] scale            # 椭球三轴半径 (rx, ry, rz)
