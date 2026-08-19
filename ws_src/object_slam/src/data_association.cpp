#include "object_slam/data_association.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace object_slam {

double DataAssociation::bboxIoU(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
  const double ix1 = std::max(a(0), b(0));
  const double iy1 = std::max(a(1), b(1));
  const double ix2 = std::min(a(2), b(2));
  const double iy2 = std::min(a(3), b(3));
  const double iw = std::max(0.0, ix2 - ix1);
  const double ih = std::max(0.0, iy2 - iy1);
  const double inter = iw * ih;

  const double areaA = (a(2) - a(0)) * (a(3) - a(1));
  const double areaB = (b(2) - b(0)) * (b(3) - b(1));
  const double uni = areaA + areaB - inter;
  if (uni <= 1e-12) return 0.0;
  return inter / uni;
}

double DataAssociation::matchingCost(const Detection2DView& d, const TrackView& track) {
  if (d.class_id != track.class_id) return std::numeric_limits<double>::infinity();
  return 1.0 - bboxIoU(d.bbox, track.bbox);
}

std::vector<Association> DataAssociation::associate(
    const std::vector<Detection2DView>& detections,
    const std::vector<TrackView>& tracks) const {
  std::vector<Association> result;
  if (tracks.empty() || detections.empty()) return result;

  std::vector<bool> det_used(detections.size(), false);

  // cost 阈值 = 1 - IoU 阈值（docs/06 §4）
  const double cost_threshold = 1.0 - iou_threshold_;

  // 贪心最近邻：对每个 track 找代价最小且未占用的检测
  for (const TrackView& track : tracks) {
    int best_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < detections.size(); ++j) {
      if (det_used[j]) continue;
      const double c = matchingCost(detections[j], track);
      if (c < best_cost) {
        best_cost = c;
        best_idx = static_cast<int>(j);
      }
    }
    // 类别一致（cost 有限）且 IoU 高于门控阈值才算匹配
    if (best_idx >= 0 && best_cost < cost_threshold) {
      det_used[best_idx] = true;
      result.push_back({track.track_id, best_idx, best_cost});
    }
  }
  return result;
  // TODO(docs/06 §3.3/§3.4): 增强 —— 加入几何线索（3D 点投影落入 mask 比例）、
  // 马氏距离门控、匈牙利算法全局最优。
}

}  // namespace object_slam
