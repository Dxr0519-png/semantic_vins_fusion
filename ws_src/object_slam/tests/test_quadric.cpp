// docs/07/08 椭球重建与联合优化 —— 单元验证。
// 覆盖：refineSingleObject 多视图 SVD 精化恢复真实椭球（docs/07 §4）、
//       单视图退化为锥不崩溃、g2o optimize() 联合优化收敛（docs/08 §5）。
//
// 用"真实椭球 -> 多视角投影 -> 精确切线 bbox"合成观测，再喂给精化/优化，
// 检查恢复的中心/半轴与真值一致。
//
// 构建运行见 docs/07 §7 / docs/08 §7：
//   colcon build --packages-select object_slam
//   ./build/object_slam/test_quadric

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "object_slam/object_map.h"
#include "object_slam/quadric.h"
#include "object_slam/quadric_optimizer.h"

using object_slam::DualQuadric;
using object_slam::ObjectMap;
using object_slam::OptimizationContext;
using object_slam::QuadricOptimizer;
using object_slam::ViewObservation;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::cerr << "CHECK failed: " #cond "  (test_quadric.cpp:" << __LINE__ \
                << ")\n";                                                    \
      std::exit(1);                                                          \
    }                                                                        \
  } while (0)

namespace {

// 相机从 cam_pos 看向 target，返回世界系相机位姿 T_wc
Eigen::Isometry3d lookAt(const Eigen::Vector3d& cam_pos,
                         const Eigen::Vector3d& target) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d z = (target - cam_pos).normalized();
  Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitY());
  if (x.norm() < 1e-6) x = z.cross(Eigen::Vector3d::UnitX());
  x.normalize();
  const Eigen::Vector3d y = z.cross(x);
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  T.linear() = R;
  T.translation() = cam_pos;
  return T;
}

// 投影对偶锥的精确轴对齐 bbox（垂直/水平切线 lᵀ C* l = 0 的根）
Eigen::Vector4d bboxOfConic(const Eigen::Matrix3d& Cs) {
  const double d = Cs(0, 2), e = Cs(1, 2), f = Cs(2, 2);
  const double disc_x = d * d - Cs(0, 0) * f;
  const double disc_y = e * e - Cs(1, 1) * f;
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  if (disc_x >= 0 && std::abs(f) > 1e-12) {
    const double sx = std::sqrt(disc_x);
    x1 = (d - sx) / f;
    x2 = (d + sx) / f;
  }
  if (disc_y >= 0 && std::abs(f) > 1e-12) {
    const double sy = std::sqrt(disc_y);
    y1 = (e - sy) / f;
    y2 = (e + sy) / f;
  }
  if (x1 > x2) std::swap(x1, x2);
  if (y1 > y2) std::swap(y1, y2);
  return Eigen::Vector4d(x1, y1, x2, y2);
}

// 相机绕物体一圈，生成 (bbox_norm, P) 观测与位姿先验
std::vector<ViewObservation> makeObservations(
    const DualQuadric& truth, const Eigen::Matrix3d& K, int W, int H,
    std::vector<std::pair<int, Eigen::Isometry3d>>& priors) {
  std::vector<ViewObservation> obs;
  const int N = 8;
  for (int i = 0; i < N; ++i) {
    const double ang = 2.0 * M_PI * i / N;
    const Eigen::Vector3d cam_pos(2.5 * std::cos(ang), 1.2 * std::sin(ang),
                                  1.5 + 0.4 * std::sin(2 * ang));
    const Eigen::Isometry3d T = lookAt(cam_pos, Eigen::Vector3d::Zero());
    const Eigen::Matrix<double, 3, 4> P = K * T.matrix().block<3, 4>(0, 0);
    const Eigen::Matrix3d Cs = P * truth.matrix() * P.transpose();
    const Eigen::Vector4d bbox_px = bboxOfConic(Cs);
    ViewObservation o;
    o.object_id = 0;
    o.frame_id = i;
    o.bbox_norm = Eigen::Vector4d(bbox_px(0) / W, bbox_px(1) / H,
                                  bbox_px(2) / W, bbox_px(3) / H);
    o.P = P;
    o.image_width = W;
    o.image_height = H;
    obs.push_back(o);
    priors.emplace_back(i, T);
  }
  return obs;
}

// 椭球半径轴序可能被特征分解打乱（eigenvalues 升序），比较时按排序后的多集对比
Eigen::Vector3d sortedRadii(const DualQuadric& q) {
  Eigen::Vector3d cq, rq;
  Eigen::Matrix3d Rq;
  q.decompose(cq, rq, Rq);
  std::sort(rq.data(), rq.data() + 3);
  return rq;
}

void compareQuadric(const DualQuadric& q, const Eigen::Vector3d& c,
                    const Eigen::Vector3d& r, double tol_c, double tol_r) {
  Eigen::Vector3d cq, rq;
  Eigen::Matrix3d Rq;
  q.decompose(cq, rq, Rq);
  std::cout << "      真值: 中心(" << c.transpose() << ") 半径(" << r.transpose()
            << ")  |  恢复: 中心(" << cq.transpose() << ") 半径("
            << rq.transpose() << ")\n";
  CHECK((cq - c).norm() < tol_c);
  Eigen::Vector3d rs = sortedRadii(q);
  Eigen::Vector3d rtrue = r;
  std::sort(rtrue.data(), rtrue.data() + 3);
  CHECK(((rs.array() - rtrue.array()) / rtrue.array()).abs().maxCoeff() < tol_r);
}

}  // namespace

int main() {
  std::cout << "docs/07/08 椭球重建验证开始...\n";
  const Eigen::Matrix3d K =
      (Eigen::Matrix3d() << 400, 0, 320, 0, 400, 240, 0, 0, 1).finished();
  const int W = 640, H = 480;
  const Eigen::Vector3d true_center(0.2, -0.1, 0.3);
  const Eigen::Vector3d true_radii(0.4, 0.3, 0.25);
  const DualQuadric truth = DualQuadric::fromEllipsoid(true_center, true_radii);

  QuadricOptimizer opt;
  std::vector<std::pair<int, Eigen::Isometry3d>> priors;
  const auto obs = makeObservations(truth, K, W, H, priors);

  // ========== docs/07 §4: 多视图 SVD 精化 ==========
  std::cout << "  [test] refineSingleObject 多视图 SVD...\n";
  const DualQuadric refined = opt.refineSingleObject(obs);
  compareQuadric(refined, true_center, true_radii, 0.02, 0.15);
  std::cout << "  [ok] 多视图 SVD 恢复真实椭球\n";

  // 单视图退回退化锥（不崩溃）
  opt.refineSingleObject({obs.front()});
  std::cout << "  [ok] 单视图退回锥（不崩溃）\n";

  // ========== docs/08: g2o 联合优化 ==========
  std::cout << "  [test] optimize() g2o 联合优化...\n";

  // 椭球误差 = 中心误差 + 三轴相对误差（轴序无关，用排序后半径）
  const auto quadricError = [&](const DualQuadric& q) {
    Eigen::Vector3d cq, rq;
    Eigen::Matrix3d Rq;
    q.decompose(cq, rq, Rq);
    std::sort(rq.data(), rq.data() + 3);
    Eigen::Vector3d rt = true_radii;
    std::sort(rt.data(), rt.data() + 3);
    return (cq - true_center).norm() +
           ((rq.array() - rt.array()).abs() / rt.array()).sum();
  };
  const auto maxPoseDrift = [](const std::vector<std::pair<int, Eigen::Isometry3d>>& refined,
                               const std::vector<std::pair<int, Eigen::Isometry3d>>& priors) {
    double mx = 0.0;
    for (size_t i = 0; i < refined.size(); ++i)
      mx = std::max(mx, (refined[i].second.translation() - priors[i].second.translation()).norm());
    return mx;
  };

  // 点-物体约束（docs/08 §3.3）：真实椭球表面采样点
  std::vector<std::pair<int, Eigen::Vector3d>> points;
  for (int i = 0; i < 60; ++i) {
    const double a = 2.0 * M_PI * i / 60.0;
    const Eigen::Vector3d n(std::cos(a), std::sin(a), 0.5 * std::sin(2 * a));
    points.emplace_back(0, true_center + true_radii.cwiseProduct(n.normalized()));
  }

  OptimizationContext ctx;
  ctx.K = K;
  ctx.image_width = W;
  ctx.image_height = H;
  ctx.max_iterations = 50;

  // 场景 A：初值差（尺度 0.8、中心偏移 ~0.15m）-> 优化应显著改善椭球
  {
    ObjectMap map;
    const int oid = map.add(0, 0.9f, DualQuadric::fromEllipsoid(
                                          true_center + Eigen::Vector3d(0.15, -0.12, 0.10),
                                          0.8 * true_radii));
    map.markObserved(oid, 0, 1);
    const double err0 = quadricError(map.get(oid)->quadric);
    std::vector<std::pair<int, Eigen::Isometry3d>> refined_poses;
    opt.optimize(map, obs, priors, refined_poses, points, ctx);
    const double err1 = quadricError(map.get(oid)->quadric);
    Eigen::Vector3d c1, r1;
    Eigen::Matrix3d R1;
    map.get(oid)->quadric.decompose(c1, r1, R1);
    std::cout << "    场景A(差初值): " << err0 << " -> " << err1
              << "  位姿最大漂移 " << maxPoseDrift(refined_poses, priors) << "\n";
    CHECK(err1 < err0 * 0.5);
    CHECK((c1 - true_center).norm() < 0.05);
    CHECK(refined_poses.size() == priors.size());
    std::cout << "  [ok] 场景A: 差初值下椭球收敛\n";
  }

  // 场景 B：初值好（refineSingleObject 结果）-> 联合优化后位姿应基本不动
  {
    ObjectMap map;
    const int oid = map.add(0, 0.9f, refined);
    map.markObserved(oid, 0, 1);
    std::vector<std::pair<int, Eigen::Isometry3d>> refined_poses;
    opt.optimize(map, obs, priors, refined_poses, points, ctx);
    const double err = quadricError(map.get(oid)->quadric);
    const double drift = maxPoseDrift(refined_poses, priors);
    std::cout << "    场景B(好初值): 椭球误差 " << err << "  位姿最大漂移 " << drift << "\n";
    CHECK(err < 0.05);                       // 椭球保持准确
    CHECK(drift < 0.05);                     // 位姿不被拉偏
    std::cout << "  [ok] 场景B: 好初值下位姿不被拉偏\n";
  }

  std::cout << "全部通过 ✓\n";
  return 0;
}
