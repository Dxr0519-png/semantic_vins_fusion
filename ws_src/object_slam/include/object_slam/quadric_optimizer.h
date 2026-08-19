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
  int frame_id = -1;                  // 检测帧/关键帧 id（docs/08 关联 SE3 顶点用）
  Eigen::Vector4d bbox_norm;          // 归一化 [xmin,ymin,xmax,ymax]
  Eigen::Matrix<double, 3, 4> P;      // 世界 -> 像素 投影 K[R|t]
  int image_width = 0;                // 归一化 bbox -> 像素 bbox 用（0 表示 bbox 已是像素）
  int image_height = 0;
};

// 联合优化的配置（docs/08）：相机内参、图像尺寸、各约束权重、迭代次数
struct OptimizationContext {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  int image_width = 640;
  int image_height = 480;
  int max_iterations = 30;
  double bbox_weight = 1.0;      // EdgeQuadricBBox 信息矩阵标量
  double pose_prior_weight = 1.0; // EdgeSE3Prior 信息矩阵标量（VINS 位姿先验，防物体约束拉偏位姿）
  double point_weight = 1.0;     // EdgePointQuadric 信息矩阵标量
};

// 物体级联合优化（g2o 因子图，见 docs/08）；单物体多视图精化（docs/07 §4）
class QuadricOptimizer {
public:
  QuadricOptimizer();
  ~QuadricOptimizer();

  // 用多视图 bbox 观测 + VINS 位姿先验，联合优化物体对偶二次曲面与关键帧位姿。
  // pose_priors: (frame_id, 世界系位姿 T_wc)；object_points: (object_id, 世界系 3D 点)。
  // refined_poses 输出优化后的位姿（回写给调用方）。
  void optimize(ObjectMap& map,
                const std::vector<ViewObservation>& obs,
                const std::vector<std::pair<int, Eigen::Isometry3d>>& pose_priors,
                std::vector<std::pair<int, Eigen::Isometry3d>>& refined_poses,
                const std::vector<std::pair<int, Eigen::Vector3d>>& object_points,
                const OptimizationContext& ctx);

  // 单物体多视图最小二乘精化（相机位姿固定），见 docs/07 §4：
  //   bbox 四边切平面约束 πᵀQ*π=0 -> 线性方程组 A x = 0 -> SVD 最小二乘 -> 投影到正椭球。
  DualQuadric refineSingleObject(const std::vector<ViewObservation>& obs);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace object_slam
