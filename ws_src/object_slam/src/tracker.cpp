#include "object_slam/tracker.h"

namespace object_slam {

Tracker::Tracker(const Eigen::Matrix3d& K, int image_width, int image_height)
    : K_(K), image_width_(image_width), image_height_(image_height) {}

void Tracker::updateFrame(const std::vector<Detection2DView>& dets,
                          const Eigen::Isometry3d& T_wc) {
  // 1) 关联线索：当前地图里所有存活 track（含短暂遮挡、尚未删除的），
  //    使重新出现的物体能匹配回原 id（docs/06 §6 重新出现）。
  std::vector<TrackView> tracks;
  tracks.reserve(map_.entries().size());
  for (const auto& [id, e] : map_.entries()) {
    TrackView t;
    t.track_id = id;
    t.class_id = e.class_id;
    t.bbox = e.bbox_smoothed;
    tracks.push_back(t);
  }

  // 2) 关联（docs/06 §3.1 类别一致 + §3.2 IoU 门控 + §4 贪心最近邻）
  const auto assoc = associator_.associate(dets, tracks);

  // 3) 匹配的 track：记录观测 + EMA 平滑 bbox（docs/06 §6 分割抖动）
  std::vector<bool> det_matched(dets.size(), false);
  for (const Association& a : assoc) {
    det_matched[a.detection_index] = true;
    ObjectEntry* e = map_.get(a.track_id);
    if (e == nullptr) continue;
    map_.markObserved(a.track_id, frame_count_, min_observations_);
    const Eigen::Vector4d& raw = dets[a.detection_index].bbox;
    e->bbox_smoothed = ema_alpha_ * raw + (1.0 - ema_alpha_) * e->bbox_smoothed;
    pushObservation(a.track_id, frame_count_, raw, T_wc);  // docs/07 §4 精化用
  }

  // 4) 新生：未匹配的检测 -> 新 track（docs/06 §2 birth；锥初始化 docs/07 §3）
  for (size_t j = 0; j < dets.size(); ++j) {
    if (det_matched[j]) continue;
    const Detection2DView& d = dets[j];
    const int id = map_.add(d.class_id, d.score, seedQuadric(d.bbox, T_wc));
    map_.markObserved(id, frame_count_, min_observations_);
    map_.get(id)->bbox_smoothed = d.bbox;
    pushObservation(id, frame_count_, d.bbox, T_wc);
  }

  // 5) 消亡：超过 max_missed_frames 帧未观测的 track 删除（docs/06 §5/§6 短暂遮挡）
  map_.prune(frame_count_, max_missed_frames_);

  // 清理已删除物体的观测缓存
  for (auto it = observations_.begin(); it != observations_.end();) {
    if (map_.get(it->first) == nullptr) {
      it = observations_.erase(it);
    } else {
      ++it;
    }
  }

  frame_count_++;
}

void Tracker::pushObservation(int object_id, int frame_id,
                              const Eigen::Vector4d& bbox_norm,
                              const Eigen::Isometry3d& T_wc) {
  ViewObservation o;
  o.object_id = object_id;
  o.frame_id = frame_id;
  o.bbox_norm = bbox_norm;
  o.P = K_ * T_wc.matrix().block<3, 4>(0, 0);
  o.image_width = image_width_;
  o.image_height = image_height_;
  std::vector<ViewObservation>& buf = observations_[object_id];
  buf.push_back(o);
  if (buf.size() > kMaxObservations) {
    buf.erase(buf.begin(), buf.begin() + (buf.size() - kMaxObservations));
  }
}

DualQuadric Tracker::seedQuadric(const Eigen::Vector4d& bbox_norm,
                                 const Eigen::Isometry3d& T_wc) const {
  // 归一化 bbox -> 像素 bbox（YOLO 节点按原图宽高归一化，docs/03）
  Eigen::Vector4d bbox_px;
  bbox_px(0) = bbox_norm(0) * static_cast<double>(image_width_);
  bbox_px(1) = bbox_norm(1) * static_cast<double>(image_height_);
  bbox_px(2) = bbox_norm(2) * static_cast<double>(image_width_);
  bbox_px(3) = bbox_norm(3) * static_cast<double>(image_height_);

  // 投影矩阵 P = K [R_wc | t_wc]（docs/07 §2）
  const Eigen::Matrix<double, 3, 4> Rt = T_wc.matrix().block<3, 4>(0, 0);
  const Eigen::Matrix<double, 3, 4> P = K_ * Rt;
  return DualQuadric::fromBBoxCone(bbox_px, P);
}

}  // namespace object_slam
