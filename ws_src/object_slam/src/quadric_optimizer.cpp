#include "object_slam/quadric_optimizer.h"

namespace object_slam {

// 用 pimpl 隐藏 g2o 头文件依赖，避免污染 node 头文件
struct QuadricOptimizer::Impl {};

QuadricOptimizer::QuadricOptimizer() : impl_(std::make_unique<Impl>()) {}
QuadricOptimizer::~QuadricOptimizer() = default;

DualQuadric QuadricOptimizer::refineSingleObject(const std::vector<ViewObservation>& obs) {
  // 思路（详见 docs/07 §4）：
  //  对每个观测 i，把 bbox 四条边反投影成切平面约束 πᵢᵀ Q* πᵢ = 0，
  //  堆叠成线性方程组 A x = 0（x 为 Q* 的 9 个独立元素），
  //  用 SVD 求最小二乘解；再施加"椭球"约束（半正定/秩约束）得到正椭球。
  // 作为骨架，这里返回从第一条观测的锥初始化结果，待按文档补全 SVD 精化。
  if (obs.empty()) return DualQuadric();
  const auto& o = obs.front();
  return DualQuadric::fromBBoxCone(o.bbox_norm, o.P);
  // TODO(docs/07 §4): 实现多视图 SVD 最小二乘 + 椭球约束投影。
}

void QuadricOptimizer::optimize(
    ObjectMap& map,
    const std::vector<ViewObservation>& obs,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& pose_priors) {
  // TODO(docs/08 §3)：构建 g2o 因子图：
  //   顶点 VertexQuadric（9 参数 Q*） + VertexSE3（可选相机位姿）
  //   边   EdgeQuadricBBox：检测 bbox 与 Q* 投影椭圆的几何误差
  //        EdgeSE3Prior：VINS 位姿先验
  //        EdgePointQuadric：点-物体关联约束（来自 VINS map_points）
  //   求解 Levenberg-Marquardt，回写 map。
  (void)map;
  (void)obs;
  (void)pose_priors;
}

}  // namespace object_slam
