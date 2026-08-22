#include "object_slam/object_slam_node.h"

#include <cmath>
#include <iterator>

#include <tf2/utils.h>

namespace object_slam {

ObjectSlamNode::ObjectSlamNode() : Node("object_slam_node") {
  // 参数（默认值与 ws_src/object_slam/config/object_slam.yaml 对齐）
  this->declare_parameter("min_observations", 3);
  this->declare_parameter("max_missed_frames", 30);
  this->declare_parameter("iou_threshold", 0.3);
  this->declare_parameter("mahalanobis_chi2_threshold", 5.991);  // docs/06 §3.4，≤0 关闭
  this->declare_parameter("ema_alpha", 0.7);
  this->declare_parameter("publish_tf", false);
  this->declare_parameter("time_tolerance", 0.05);  // 检测帧与关键帧最大时间差(秒)
  this->declare_parameter("camera_fx", 426.0);
  this->declare_parameter("camera_fy", 426.0);
  this->declare_parameter("camera_cx", 320.0);
  this->declare_parameter("camera_cy", 240.0);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("image_height", 480);
  this->declare_parameter("refine_interval", 1);
  this->declare_parameter("optimize_interval", 20);
  this->declare_parameter("optimize_iters", 30);
  this->get_parameter("time_tolerance", time_tolerance_);
  this->get_parameter("min_observations", min_observations_);
  this->get_parameter("max_missed_frames", max_missed_frames_);
  this->get_parameter("iou_threshold", iou_threshold_);
  this->get_parameter("mahalanobis_chi2_threshold", mahalanobis_chi2_threshold_);
  this->get_parameter("ema_alpha", ema_alpha_);
  this->get_parameter("camera_fx", camera_fx_);
  this->get_parameter("camera_fy", camera_fy_);
  this->get_parameter("camera_cx", camera_cx_);
  this->get_parameter("camera_cy", camera_cy_);
  this->get_parameter("image_width", image_width_);
  this->get_parameter("image_height", image_height_);
  this->get_parameter("refine_interval", refine_interval_);
  this->get_parameter("optimize_interval", optimize_interval_);
  this->get_parameter("optimize_iters", optimize_iters_);

  optimizer_ = std::make_unique<QuadricOptimizer>();

  // 跟踪器：相机内参用于新 track 锥初始化（docs/07 §3）
  Eigen::Matrix3d K;
  K << camera_fx_, 0.0, camera_cx_,
       0.0, camera_fy_, camera_cy_,
       0.0, 0.0, 1.0;
  tracker_ = std::make_unique<Tracker>(K, image_width_, image_height_);
  tracker_->setMinObservations(min_observations_);
  tracker_->setMaxMissedFrames(max_missed_frames_);
  tracker_->setIoUThreshold(iou_threshold_);
  tracker_->setChi2Threshold(mahalanobis_chi2_threshold_);
  tracker_->setEmaAlpha(ema_alpha_);

  // 订阅：YOLO 检测（Best Effort 对齐相机 QoS）
  auto be = rclcpp::QoS(1).best_effort();
  det_sub_ = this->create_subscription<semantic_interfaces::msg::Detection2DArray>(
      "/yolo/detections", be,
      std::bind(&ObjectSlamNode::detectionCallback, this, std::placeholders::_1));

  // 位姿/点来自 VINS vins_node 实际发布的话题（见 vins/src/utility/visualization.cpp 的
  // registerPub：相对名 keyframe_pose / keyframe_point，解析到 / 命名空间）
  pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/keyframe_pose", 10,
      std::bind(&ObjectSlamNode::poseCallback, this, std::placeholders::_1));

  pts_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "/keyframe_point", 10,
      std::bind(&ObjectSlamNode::pointsCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<semantic_interfaces::msg::ObjectMap>(
      "/object_slam/objects", 10);
  traj_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/object_slam/trajectory", 10);

  RCLCPP_INFO(this->get_logger(), "object_slam_node 启动");
}

void ObjectSlamNode::detectionCallback(
    const semantic_interfaces::msg::Detection2DArray::SharedPtr msg) {
  // 0) 时间同步（docs/04 §3.2 方案 A）：找该检测帧最近的 VINS 关键帧位姿 T_wc
  rclcpp::Time t_det(msg->header.stamp);
  auto match = nearestKeyframe(t_det);
  if (match.pose == nullptr) {
    // VINS 还没出任何关键帧，拿不到位姿，跳过（避免用错位姿）
    RCLCPP_DEBUG(this->get_logger(), "VINS 尚无关键帧，跳过该检测帧");
    return;
  }
  const double dt = (t_det - match.time).seconds();
  if (std::abs(dt) > time_tolerance_) {
    RCLCPP_DEBUG(this->get_logger(),
                 "检测帧与关键帧时间差 %.3f s 超容差 %.3f s，跳过",
                 dt, time_tolerance_);
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "检测帧 %.3f s -> 关键帧 %.3f s (dt=%.1f ms)，dets=%zu，pos=(%.2f %.2f %.2f)",
              t_det.seconds(), match.time.seconds(), dt * 1e3,
              msg->detections.size(),
              match.pose->translation().x(), match.pose->translation().y(),
              match.pose->translation().z());

  // 1) 转成 Detection2DView
  std::vector<Detection2DView> dets;
  for (const auto& d : msg->detections) {
    Detection2DView v;
    v.class_id = d.class_id;
    v.score = d.score;
    v.bbox << d.x_min, d.y_min, d.x_max, d.y_max;
    dets.push_back(v);
  }

  // 2) 帧级跟踪：关联 + 匹配(EMA) + 新生(锥初始化) + 消亡（docs/06）
  tracker_->updateFrame(dets, *match.pose);

  // 记录该检测帧对应的关键帧位姿（docs/08 联合优化的位姿先验，与观测 frame_id 对齐）
  const int cur_frame = tracker_->frame() - 1;
  pose_by_frame_[cur_frame] = *match.pose;

  // 3) 物体重建（docs/07 §4）：用多视图 bbox 观测精化各物体椭球
  if (cur_frame % refine_interval_ == 0) refineObjects();

  // 4) 物体级联合优化（docs/08 §5）：周期性触发 g2o 因子图优化
  if (optimize_interval_ > 0 && cur_frame % optimize_interval_ == 0) {
    runJointOptimization();
  }

  publishObjectMap();
}

KeyframeMatch ObjectSlamNode::nearestKeyframe(rclcpp::Time t) const {
  KeyframeMatch m;
  if (keyframes_.empty()) return m;

  auto it = keyframes_.lower_bound(t);  // 第一个时间戳 >= t 的关键帧
  if (it == keyframes_.end()) {         // t 晚于所有关键帧 -> 取最后一帧
    it = std::prev(it);
    m.time = it->first;
    m.pose = &it->second;
    return m;
  }
  if (it == keyframes_.begin()) {       // t 早于所有关键帧 -> 取第一帧
    m.time = it->first;
    m.pose = &it->second;
    return m;
  }
  // t 落在两帧之间 -> 取时间更近的一帧
  auto prev = std::prev(it);
  const bool take_prev =
      (t - prev->first).nanoseconds() <= (it->first - t).nanoseconds();
  m.time = take_prev ? prev->first : it->first;
  m.pose = take_prev ? &prev->second : &it->second;
  return m;
}

void ObjectSlamNode::poseCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // VINS keyframe_pose 是 nav_msgs/Odometry，pose 即世界系相机位姿 T_wc
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  T.linear() = q.toRotationMatrix();
  keyframes_[msg->header.stamp] = T;
}

void ObjectSlamNode::pointsCallback(
    const sensor_msgs::msg::PointCloud::SharedPtr msg) {
  // 缓存 VINS 世界系 3D 点（docs/07 §5 点-物体关联用），只保留最近 kMaxWorldPoints
  constexpr size_t kMaxWorldPoints = 500;
  for (const auto& p : msg->points) {
    world_points_.emplace_back(p.x, p.y, p.z);
  }
  if (world_points_.size() > kMaxWorldPoints) {
    world_points_.erase(world_points_.begin(),
                        world_points_.begin() + (world_points_.size() - kMaxWorldPoints));
  }
}

void ObjectSlamNode::refineObjects() {
  for (const auto& [oid, obs] : tracker_->observations()) {
    if (obs.size() < 2) continue;  // 单视图仍退化，保留锥
    DualQuadric q = optimizer_->refineSingleObject(obs);
    tracker_->map().update(oid, q);
  }
}

void ObjectSlamNode::runJointOptimization() {
  if (pose_by_frame_.size() < 2) return;

  // 收集所有物体的多视图观测
  std::vector<ViewObservation> all_obs;
  for (const auto& [oid, obs] : tracker_->observations()) {
    all_obs.insert(all_obs.end(), obs.begin(), obs.end());
  }
  if (all_obs.empty()) return;

  // 位姿先验（按帧号升序）
  std::vector<std::pair<int, Eigen::Isometry3d>> priors(pose_by_frame_.begin(),
                                                        pose_by_frame_.end());

  // 点-物体关联（docs/07 §5）：世界点按 signedDistance 最近邻归属物体
  std::vector<std::pair<int, Eigen::Vector3d>> object_points;
  associatePointsToObjects(object_points);

  OptimizationContext ctx;
  ctx.K << camera_fx_, 0.0, camera_cx_,
          0.0, camera_fy_, camera_cy_,
          0.0, 0.0, 1.0;
  ctx.image_width = image_width_;
  ctx.image_height = image_height_;
  ctx.max_iterations = optimize_iters_;

  std::vector<std::pair<int, Eigen::Isometry3d>> refined_poses;
  optimizer_->optimize(tracker_->map(), all_obs, priors, refined_poses,
                       object_points, ctx);

  // 回写精化后的位姿（后续联合优化用它做先验）
  for (const auto& [fid, T] : refined_poses) {
    if (pose_by_frame_.count(fid)) pose_by_frame_[fid] = T;
  }
}

void ObjectSlamNode::associatePointsToObjects(
    std::vector<std::pair<int, Eigen::Vector3d>>& out) {
  if (world_points_.empty()) return;
  const double kAssocThreshold = 0.5;  // 归一化距离 ≤ 0.5 视为属于该物体
  constexpr int kMaxPointsPerObject = 30;
  std::map<int, int> per_object_count;
  for (const Eigen::Vector3d& p : world_points_) {
    int best_oid = -1;
    double best_sd = kAssocThreshold;
    for (const auto& [oid, e] : tracker_->map().entries()) {
      if (!e.valid) continue;
      const double sd = e.quadric.signedDistance(p);
      if (sd < best_sd) {
        best_sd = sd;
        best_oid = oid;
      }
    }
    if (best_oid >= 0 && per_object_count[best_oid] < kMaxPointsPerObject) {
      out.emplace_back(best_oid, p);
      per_object_count[best_oid]++;
    }
  }
}

void ObjectSlamNode::publishObjectMap() {
  semantic_interfaces::msg::ObjectMap out;
  out.header.stamp = this->now();
  out.header.frame_id = "world";

  for (const auto& [id, e] : tracker_->map().entries()) {
    if (!e.valid) continue;
    semantic_interfaces::msg::Object3D o;
    o.id = e.id;
    o.class_id = e.class_id;
    o.confidence = e.confidence;

    // Q*（行主序 16 元素）
    const Eigen::Matrix4d& Q = e.quadric.matrix();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        o.dual_quadric[4 * r + c] = Q(r, c);

    Eigen::Vector3d center, radii;
    Eigen::Matrix3d R;
    e.quadric.decompose(center, radii, R);
    o.center.x = center.x(); o.center.y = center.y(); o.center.z = center.z();
    o.scale[0] = radii.x(); o.scale[1] = radii.y(); o.scale[2] = radii.z();

    out.objects.push_back(o);
  }
  map_pub_->publish(out);
}

}  // namespace object_slam

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<object_slam::ObjectSlamNode>());
  rclcpp::shutdown();
  return 0;
}
