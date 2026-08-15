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
  // 把当前帧检测与已有 track 关联（贪心最近邻 + 类别一致）
  std::vector<Association> associate(
      const std::vector<Detection2DView>& detections,
      const std::vector<TrackView>& tracks) const;

  // bbox IoU（归一化坐标下直接算）
  static double bboxIoU(const Eigen::Vector4d& a, const Eigen::Vector4d& b);

  // 匹配代价：类别不一致返回 inf；否则 1 - IoU
  static double matchingCost(const Detection2DView& d, const TrackView& track);

  int nextTrackId() { return next_id_++; }

private:
  int next_id_ = 0;
};

}  // namespace object_slam
