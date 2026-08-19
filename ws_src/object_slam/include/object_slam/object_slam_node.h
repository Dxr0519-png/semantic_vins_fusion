#pragma once

#include <Eigen/Geometry>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <semantic_interfaces/msg/detection2_d_array.hpp>
#include <semantic_interfaces/msg/object_map.hpp>

#include "object_slam/quadric_optimizer.h"
#include "object_slam/tracker.h"

namespace object_slam {

// 关键帧时间匹配结果（docs/04 §3.2 方案 A：检测帧时间戳 -> 最近关键帧）
struct KeyframeMatch {
  rclcpp::Time time;                        // 命中的关键帧时间戳
  const Eigen::Isometry3d* pose = nullptr;  // 世界系相机位姿 T_wc；无帧时为 nullptr
};

// 物体级 SLAM 主节点：串联检测/位姿/点云，输出物体地图（见 docs/06~08、10）
class ObjectSlamNode : public rclcpp::Node {
public:
  ObjectSlamNode();

private:
  void detectionCallback(const semantic_interfaces::msg::Detection2DArray::SharedPtr msg);
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pointsCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

  void publishObjectMap();

  // docs/07 §4：用多视图 bbox 观测精化各物体椭球
  void refineObjects();

  // docs/08：周期性触发 g2o 联合优化（物体椭球 + 关键帧位姿）
  void runJointOptimization();

  // docs/07 §5：世界系 3D 点按 signedDistance 最近邻归属到物体（作为点约束）
  void associatePointsToObjects(std::vector<std::pair<int, Eigen::Vector3d>>& out);

  // 在关键帧时间序列里找与 t 最近的一帧（std::lower_bound 二分）
  KeyframeMatch nearestKeyframe(rclcpp::Time t) const;

  // 关键帧位姿缓存（时间戳 -> 世界系相机位姿 T_wc）
  std::map<rclcpp::Time, Eigen::Isometry3d> keyframes_;

  // 检测帧号 -> 匹配到的关键帧位姿（docs/08 联合优化的位姿先验，与 Tracker 的观测 frame_id 对齐）
  std::map<int, Eigen::Isometry3d> pose_by_frame_;

  // 最近缓存的 VINS 世界系 3D 点（docs/07 §5 点-物体关联用）
  std::vector<Eigen::Vector3d> world_points_;

  std::unique_ptr<QuadricOptimizer> optimizer_;  // 物体联合优化（docs/07、08）
  std::unique_ptr<Tracker> tracker_;             // 帧级跟踪（docs/06）

  // 检测帧与关键帧允许的最大时间差（秒），超差则丢弃该帧
  double time_tolerance_ = 0.05;

  // 跟踪参数（docs/06 §5/§6）
  int min_observations_ = 3;
  int max_missed_frames_ = 30;
  double iou_threshold_ = 0.3;
  double ema_alpha_ = 0.7;

  // 优化调度（docs/07 §4 / docs/08 §5）
  int refine_interval_ = 1;      // 每 N 帧精化一次物体椭球
  int optimize_interval_ = 20;   // 每 N 帧触发一次 g2o 联合优化
  int optimize_iters_ = 30;      // g2o 迭代次数

  // 相机内参 + 图像尺寸：新 track 锥初始化（docs/07 §3），占位值需按 docs/04 标定替换
  double camera_fx_ = 426.0;
  double camera_fy_ = 426.0;
  double camera_cx_ = 320.0;
  double camera_cy_ = 240.0;
  int image_width_ = 640;
  int image_height_ = 480;

  rclcpp::Subscription<semantic_interfaces::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pts_sub_;
  rclcpp::Publisher<semantic_interfaces::msg::ObjectMap>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
};

}  // namespace object_slam
