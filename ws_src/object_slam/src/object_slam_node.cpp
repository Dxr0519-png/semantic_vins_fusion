#include "object_slam/object_slam_node.h"

#include <tf2/utils.h>

namespace object_slam {

ObjectSlamNode::ObjectSlamNode() : Node("object_slam_node") {
  // 参数
  this->declare_parameter("min_observations", 3);
  this->declare_parameter("max_missed_frames", 30);
  this->declare_parameter("publish_tf", false);

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
  (void)msg;
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
