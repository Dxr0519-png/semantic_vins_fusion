#include "object_slam/object_slam_node.h"

#include <cmath>
#include <iterator>

#include <tf2/utils.h>

namespace object_slam {

ObjectSlamNode::ObjectSlamNode() : Node("object_slam_node") {
  // 参数
  this->declare_parameter("min_observations", 3);
  this->declare_parameter("max_missed_frames", 30);
  this->declare_parameter("publish_tf", false);
  this->declare_parameter("time_tolerance", 0.05);  // 检测帧与关键帧最大时间差(秒)
  this->get_parameter("time_tolerance", time_tolerance_);

  optimizer_ = std::make_unique<QuadricOptimizer>();

  // 订阅：YOLO 检测（Best Effort 对齐相机 QoS）
  auto be = rclcpp::QoS(1).best_effort();
  det_sub_ = this->create_subscription<semantic_interfaces::msg::Detection2DArray>(
      "/yolo/detections", be,
      std::bind(&ObjectSlamNode::detectionCallback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vins_fusion/keyframe_pose", 10,
      std::bind(&ObjectSlamNode::poseCallback, this, std::placeholders::_1));

  pts_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "/vins_fusion/map_points", 10,
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

  // 2) 关联（复用上一帧 track）
  auto assoc = associator_.associate(dets, last_tracks_);

  // 3) 更新 / 新建物体
  //    TODO(docs/06 §3): 未匹配检测 -> 新 track（add + 锥初始化，需该帧位姿）
  //    TODO(docs/07 §4): 匹配检测 -> refineSingleObject 精化椭球
  //    TODO(docs/08 §3): 定期调用 optimizer_->optimize(...)

  // 4) 更新 last_tracks_ 供下一帧
  last_tracks_.clear();
  for (const auto& d : dets) {
    TrackView t;
    t.track_id = 0;  // TODO: 填真实 id
    t.class_id = d.class_id;
    t.bbox = d.bbox;
    last_tracks_.push_back(t);
  }

  frame_count_++;
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
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);
  T.linear() = q.toRotationMatrix();
  keyframes_[msg->header.stamp] = T;
}

void ObjectSlamNode::pointsCallback(
    const sensor_msgs::msg::PointCloud::SharedPtr msg) {
  // TODO(docs/07 §5): 缓存世界系 3D 点，用于"点落在物体 mask 内"的关联与几何约束。
  (void)msg;
}

void ObjectSlamNode::publishObjectMap() {
  semantic_interfaces::msg::ObjectMap out;
  out.header.stamp = this->now();
  out.header.frame_id = "world";

  for (const auto& [id, e] : map_.entries()) {
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
