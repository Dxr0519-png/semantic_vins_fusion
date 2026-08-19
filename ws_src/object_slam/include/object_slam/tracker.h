#pragma once

#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "object_slam/data_association.h"
#include "object_slam/object_map.h"
#include "object_slam/quadric.h"
#include "object_slam/quadric_optimizer.h"

namespace object_slam {

// 帧级物体跟踪器（docs/06）：喂一帧检测 + 相机位姿，维护跨帧稳定的 track_id。
// 纯逻辑、无 ROS 依赖，便于单元测试（tests/test_tracking.cpp）。
class Tracker {
public:
  // K: 相机内参（新 track 锥初始化用，docs/07 §3）；
  // image_width/height: 原图像尺寸，把归一化 bbox 还原成像素坐标。
  Tracker(const Eigen::Matrix3d& K, int image_width, int image_height);

  // 参数（docs/06 §5/§6）
  void setMinObservations(int n) { min_observations_ = n; }
  void setMaxMissedFrames(int n) { max_missed_frames_ = n; }
  void setIoUThreshold(double t) { associator_.setIoUThreshold(t); }
  void setEmaAlpha(double a) { ema_alpha_ = a; }

  // 处理一帧检测：
  //   1) 关联（类别一致 + IoU 门控 + 贪心最近邻）
  //   2) 匹配的 track -> markObserved + EMA 平滑 bbox
  //   3) 未匹配检测 -> 新生 track（锥初始化）
  //   4) 超过 max_missed_frames 帧未观测 -> 消亡
  // dets: 本帧检测（归一化 bbox）；T_wc: 该检测帧匹配到的关键帧相机位姿
  void updateFrame(const std::vector<Detection2DView>& dets,
                   const Eigen::Isometry3d& T_wc);

  const ObjectMap& map() const { return map_; }
  ObjectMap& map() { return map_; }

  // 当前帧号（已处理帧数 - 1 = 上一帧；驱动 docs/07 精化 / docs/08 优化的调度）
  int frame() const { return frame_count_; }

  // 每个物体累积的多视图观测（bbox + 投影矩阵），供 docs/07 §4 精化与 docs/08 联合优化
  const std::map<int, std::vector<ViewObservation>>& observations() const {
    return observations_;
  }

private:
  // 记录一次观测（bound 到 kMaxObservations 帧/物体）
  void pushObservation(int object_id, int frame_id,
                       const Eigen::Vector4d& bbox_norm,
                       const Eigen::Isometry3d& T_wc);

  // 从归一化 bbox + 位姿初始化对偶锥（退化的秩 3 Q*，仅作初值，docs/07 §3 精化）
  DualQuadric seedQuadric(const Eigen::Vector4d& bbox_norm,
                          const Eigen::Isometry3d& T_wc) const;

  static constexpr int kMaxObservations = 60;

  DataAssociation associator_;
  ObjectMap map_;
  std::map<int, std::vector<ViewObservation>> observations_;  // object_id -> 观测序列
  Eigen::Matrix3d K_;
  int image_width_;
  int image_height_;
  int frame_count_ = 0;  // 已处理的检测帧计数（驱动 prune 的"当前帧"）
  int min_observations_ = 3;
  int max_missed_frames_ = 30;
  double ema_alpha_ = 0.7;  // 检测 bbox 的 EMA 权重（docs/06 §6 分割抖动平滑）
};

}  // namespace object_slam
