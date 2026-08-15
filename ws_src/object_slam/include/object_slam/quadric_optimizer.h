#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <utility>
#include <vector>

#include "object_slam/object_map.h"
#include "object_slam/quadric.h"

namespace object_slam {

// 一次观测：某帧对某物体的 2D bbox + 该帧相机投影矩阵
struct ViewObservation {
  int object_id;
  Eigen::Vector4d bbox_norm;         // 归一化 [xmin,ymin,xmax,ymax]
  Eigen::Matrix<double, 3, 4> P;     // 世界 -> 像素 投影 K[R|t]
};

// 物体级联合优化（g2o 因子图，见 docs/08）
class QuadricOptimizer {
public:
  QuadricOptimizer();
  ~QuadricOptimizer();

  // 用多视图 bbox 观测 + VINS 位姿先验，优化物体对偶二次曲面（可扩展联调位姿）
  // pose_priors: (frame_id, 世界系位姿 T_wc)
  void optimize(ObjectMap& map,
                const std::vector<ViewObservation>& obs,
                const std::vector<std::pair<int, Eigen::Isometry3d>>& pose_priors);

  // 单物体多视图最小二乘精化（相机位姿固定），见 docs/07 §4
  DualQuadric refineSingleObject(const std::vector<ViewObservation>& obs);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace object_slam
