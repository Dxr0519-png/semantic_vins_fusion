#pragma once

#include <Eigen/Core>
#include <vector>

namespace object_slam {

// 归一化 2D 检测视图（由 ROS 消息转换而来，解耦 ROS 依赖）
struct Detection2DView {
  int class_id;
  Eigen::Vector4d bbox;  // 归一化 [xmin, ymin, xmax, ymax]
  float score;
};

// 已跟踪物体的上一帧状态（用于关联）
struct TrackView {
  int track_id;
  int class_id;
  Eigen::Vector4d bbox;  // 归一化
  // 世界系对偶二次曲面 Q*（椭球，docs/07）。零矩阵 = 无有效椭球
  // （如新 track 的秩 3 种子锥尚未精化）-> 跳过马氏门控，退化为纯 IoU（docs/06 §3.4）。
  Eigen::Matrix4d quadric = Eigen::Matrix4d::Zero();
};

// 一帧关联结果
struct Association {
  int track_id;         // 已存在的 track
  int detection_index;  // 对应本帧 detections 的下标
  double cost;          // 越小越好
};

// 物体级数据关联：跨帧维护 track_id（见 docs/06）
class DataAssociation {
public:
  // IoU 门控阈值：只接受 IoU 高于该值的匹配（docs/06 §4 "若 cost < 阈值"）。
  // cost = 1 - IoU，故等价于 cost < 1 - iou_threshold。
  void setIoUThreshold(double t) { iou_threshold_ = t; }

  // 马氏距离门控阈值（docs/06 §3.4）：d_M² ≥ 该值的检测视为几何上不可能的匹配，拒绝。
  // 默认 5.991 = χ²₂,₀.₉₅；设 ≤ 0 时整体关闭马氏门控（退化为纯 IoU 关联）。
  void setChi2Threshold(double t) { chi2_threshold_ = t; }

  // 把当前帧检测与已有 track 关联（类别一致 + IoU 门控 + 马氏距离门控 + 贪心最近邻）。
  // P: 当前帧完整投影矩阵 K[R_wc | t_wc]（世界 -> 像素，docs/07 §2.3），用于把 track 的
  //    椭球投影到当前帧做马氏门控；image_width/height: 归一化 bbox 中心 -> 像素坐标。
  // 无有效椭球（TrackView::quadric 为零矩阵或非正椭球）的 track 跳过马氏门控。
  std::vector<Association> associate(
      const std::vector<Detection2DView>& detections,
      const std::vector<TrackView>& tracks,
      const Eigen::Matrix<double, 3, 4>& P,
      int image_width, int image_height) const;

  // bbox IoU（归一化坐标下直接算）
  static double bboxIoU(const Eigen::Vector4d& a, const Eigen::Vector4d& b);

  // 匹配代价：类别不一致返回 inf；否则 1 - IoU
  static double matchingCost(const Detection2DView& d, const TrackView& track);

private:
  double iou_threshold_ = 0.3;
  double chi2_threshold_ = 5.991;  // 马氏距离门控阈值（χ²₂,₀.₉₅），≤0 关闭
};

}  // namespace object_slam
