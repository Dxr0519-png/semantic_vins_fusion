#pragma once

#include <Eigen/Geometry>
#include <map>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <semantic_interfaces/msg/detection2_d_array.hpp>
#include <semantic_interfaces/msg/object_map.hpp>

#include "object_slam/data_association.h"
#include "object_slam/object_map.h"
#include "object_slam/quadric_optimizer.h"

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
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pointsCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

  void publishObjectMap();

  // 在关键帧时间序列里找与 t 最近的一帧（std::lower_bound 二分）
  KeyframeMatch nearestKeyframe(rclcpp::Time t) const;

  // 关键帧位姿缓存（时间戳 -> 世界系相机位姿 T_wc）
  std::map<rclcpp::Time, Eigen::Isometry3d> keyframes_;

  DataAssociation associator_;
  ObjectMap map_;
  std::unique_ptr<QuadricOptimizer> optimizer_;

  // 上一帧各 track 的 bbox（用于帧间关联）
  std::vector<TrackView> last_tracks_;

  int frame_count_ = 0;

  // 检测帧与关键帧允许的最大时间差（秒），超差则丢弃该帧
  double time_tolerance_ = 0.05;

  rclcpp::Subscription<semantic_interfaces::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr pts_sub_;
  rclcpp::Publisher<semantic_interfaces::msg::ObjectMap>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
};

}  // namespace object_slam
