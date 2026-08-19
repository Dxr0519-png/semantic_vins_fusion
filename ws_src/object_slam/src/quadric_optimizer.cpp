#include "object_slam/quadric_optimizer.h"

#include <cmath>
#include <map>
#include <set>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace object_slam {

// 用 pimpl 隐藏 g2o 头文件依赖，避免污染 node 头文件
struct QuadricOptimizer::Impl {};

QuadricOptimizer::QuadricOptimizer() : impl_(std::make_unique<Impl>()) {}
QuadricOptimizer::~QuadricOptimizer() = default;

namespace {

// 归一化 bbox -> 像素 bbox（image 尺寸为 0 时视为已是像素坐标）
Eigen::Vector4d bboxToPixels(const Eigen::Vector4d& b, int w, int h) {
  if (w <= 0 || h <= 0) return b;
  return Eigen::Vector4d(b(0) * w, b(1) * h, b(2) * w, b(3) * h);
}

// =====================================================================
// docs/08 §4 —— 自定义 g2o 顶点/边
// =====================================================================

// Q* 的 9 个自由参数 = 上三角(不含 (3,3))；oplus 只改这 9 个，且 (3,3) 固定为 -1
// 作为尺度规范（与 docs/07 §2.4 decompose 的归一化一致）。
class VertexQuadric : public g2o::BaseVertex<9, Eigen::Matrix4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int kIdx[9][2];
  VertexQuadric() { _estimate = Eigen::Matrix4d::Zero(); }
  void setToOriginImpl() override { _estimate = DualQuadric().matrix(); }
  void oplusImpl(const double* update) override {
    for (int i = 0; i < 9; ++i) {
      _estimate(kIdx[i][0], kIdx[i][1]) += update[i];
      if (kIdx[i][0] != kIdx[i][1]) _estimate(kIdx[i][1], kIdx[i][0]) += update[i];
    }
    _estimate(3, 3) = -1.0;
  }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }
};
const int VertexQuadric::kIdx[9][2] = {{0,0},{0,1},{0,2},{0,3},{1,1},{1,2},{1,3},{2,2},{2,3}};

// 核心约束边（docs/08 §3.1）：投影对偶锥 C* = P Q* Pᵀ 与 bbox 四条边线的切平面
// 代数距离 e_k = l_kᵀ C* l_k（l_k 为像素空间单位法向直线，C* 归一化）。
class EdgeQuadricBBox
    : public g2o::BaseBinaryEdge<4, Eigen::Vector4d, VertexQuadric, g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
  int image_width_ = 640;
  int image_height_ = 480;

  void computeError() override {
    const VertexQuadric* vq = static_cast<const VertexQuadric*>(_vertices[0]);
    const g2o::VertexSE3* vp = static_cast<const g2o::VertexSE3*>(_vertices[1]);
    // 在归一化图像坐标下计算（P = [R|t] 不含 K，bbox 除以内参），
    // 使误差/梯度尺度 O(1)，LM 收敛稳定（像素坐标下 K~500 会让梯度爆掉）。
    const Eigen::Matrix<double, 3, 4> P =
        vp->estimate().matrix().block<3, 4>(0, 0);
    Eigen::Matrix3d Cs = P * vq->estimate() * P.transpose();
    Cs /= Cs.norm();

    const Eigen::Vector4d& b = _measurement;  // 归一化 bbox [0,1]
    const double fx = K_(0, 0), fy = K_(1, 1);
    const double cx = K_(0, 2), cy = K_(1, 2);
    const double x1 = (b(0) * image_width_ - cx) / fx;
    const double x2 = (b(2) * image_width_ - cx) / fx;
    const double y1 = (b(1) * image_height_ - cy) / fy;
    const double y2 = (b(3) * image_height_ - cy) / fy;
    Eigen::Matrix<double, 3, 4> lines;
    lines.col(0) << 1, 0, -x1;  // left
    lines.col(1) << 1, 0, -x2;  // right
    lines.col(2) << 0, 1, -y1;  // top
    lines.col(3) << 0, 1, -y2;  // bottom
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector3d l = lines.col(k);
      const double n2 = l(0) * l(0) + l(1) * l(1);
      const Eigen::Vector3d lu = n2 > 1e-12 ? l / std::sqrt(n2) : l;
      _error(k) = lu.dot(Cs * lu);
    }
  }

  // 数值中心差分（quadric 9 参数 + SE3 6 参数），避免手推解析雅可比
  void linearizeOplus() override {
    const double eps = 1e-6;
    VertexQuadric* vq = static_cast<VertexQuadric*>(_vertices[0]);
    g2o::VertexSE3* vp = static_cast<g2o::VertexSE3*>(_vertices[1]);
    const Eigen::Matrix4d Q0 = vq->estimate();
    const Eigen::Isometry3d T0 = vp->estimate();

    Eigen::Matrix<double, 4, 9> Jq;
    for (int i = 0; i < 9; ++i) {
      Eigen::Matrix4d Qp = Q0, Qm = Q0;
      Qp(VertexQuadric::kIdx[i][0], VertexQuadric::kIdx[i][1]) += eps;
      Qm(VertexQuadric::kIdx[i][0], VertexQuadric::kIdx[i][1]) -= eps;
      if (VertexQuadric::kIdx[i][0] != VertexQuadric::kIdx[i][1]) {
        Qp(VertexQuadric::kIdx[i][1], VertexQuadric::kIdx[i][0]) += eps;
        Qm(VertexQuadric::kIdx[i][1], VertexQuadric::kIdx[i][0]) -= eps;
      }
      vq->setEstimate(Qp); computeError(); const Eigen::Vector4d ep = _error;
      vq->setEstimate(Qm); computeError(); const Eigen::Vector4d em = _error;
      Jq.col(i) = (ep - em) / (2.0 * eps);
    }
    vq->setEstimate(Q0);

    Eigen::Matrix<double, 4, 6> Jp;
    for (int i = 0; i < 6; ++i) {
      Eigen::Matrix<double, 6, 1> d = Eigen::Matrix<double, 6, 1>::Zero();
      d(i) = eps;
      const Eigen::Isometry3d inc = g2o::internal::fromVectorMQT(d);
      vp->setEstimate(T0 * inc);
      computeError();
      const Eigen::Vector4d ep = _error;
      vp->setEstimate(T0 * inc.inverse());
      computeError();
      const Eigen::Vector4d em = _error;
      Jp.col(i) = (ep - em) / (2.0 * eps);
    }
    vp->setEstimate(T0);

    _jacobianOplusXi = Jq;
    _jacobianOplusXj = Jp;
  }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }
};

// SE3 位姿先验边（docs/08 §3.2）：右不变误差 e = log(T_est · T_prior⁻¹)。
// 雅可比在先验附近 ≈ 单位阵（VINS 初值通常很接近，取近似即可）。
class EdgeSE3Prior
    : public g2o::BaseUnaryEdge<6, Eigen::Isometry3d, g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void computeError() override {
    const g2o::VertexSE3* v = static_cast<const g2o::VertexSE3*>(_vertices[0]);
    _error = g2o::internal::toVectorMQT(v->estimate() * _measurement.inverse());
  }
  void linearizeOplus() override { _jacobianOplusXi.setIdentity(); }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }
};

// 点-物体约束边（docs/08 §3.3 / docs/07 §5）：误差 = max(0, signedDistance(p))，
// 只惩罚椭球外部的点（表面/内部点误差为 0，防止椭球被拉缩）。
class EdgePointQuadric
    : public g2o::BaseUnaryEdge<1, double, VertexQuadric> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  void computeError() override {
    const VertexQuadric* vq = static_cast<const VertexQuadric*>(_vertices[0]);
    const double sd = DualQuadric(vq->estimate()).signedDistance(p_);
    _error(0) = sd > 0.0 ? sd : 0.0;
  }
  void linearizeOplus() override {
    const double eps = 1e-5;
    VertexQuadric* vq = static_cast<VertexQuadric*>(_vertices[0]);
    const Eigen::Matrix4d Q0 = vq->estimate();
    Eigen::Matrix<double, 1, 9> J;
    for (int i = 0; i < 9; ++i) {
      Eigen::Matrix4d Qp = Q0, Qm = Q0;
      Qp(VertexQuadric::kIdx[i][0], VertexQuadric::kIdx[i][1]) += eps;
      Qm(VertexQuadric::kIdx[i][0], VertexQuadric::kIdx[i][1]) -= eps;
      if (VertexQuadric::kIdx[i][0] != VertexQuadric::kIdx[i][1]) {
        Qp(VertexQuadric::kIdx[i][1], VertexQuadric::kIdx[i][0]) += eps;
        Qm(VertexQuadric::kIdx[i][1], VertexQuadric::kIdx[i][0]) -= eps;
      }
      vq->setEstimate(Qp); computeError(); const double ep = _error(0);
      vq->setEstimate(Qm); computeError(); const double em = _error(0);
      J(0, i) = (ep - em) / (2.0 * eps);
    }
    vq->setEstimate(Q0);
    _jacobianOplusXi = J;
  }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }
};

}  // namespace

// =====================================================================
// docs/07 §4 —— 单物体多视图 SVD 最小二乘精化
// =====================================================================
DualQuadric QuadricOptimizer::refineSingleObject(
    const std::vector<ViewObservation>& obs) {
  if (obs.empty()) return DualQuadric();

  const ViewObservation& first = obs.front();
  const auto fallback_cone = [&first]() {
    return DualQuadric::fromBBoxCone(
        bboxToPixels(first.bbox_norm, first.image_width, first.image_height),
        first.P);
  };

  // 单视图：深度不可观，只能给退化锥（docs/07 §3）
  if (obs.size() < 2) return fallback_cone();

  // 每个观测的 bbox 四边 -> 4 个切平面约束：lᵀ P Q* Pᵀ l = 0
  //   令 m = Pᵀ l（世界系对偶线），则约束为 mᵀ Q* m = 0，是 Q* 的线性方程。
  // 堆叠成齐次方程组 M q = 0（M ∈ R^(4K × 10)），最小二乘解 = M 的最小右奇异向量。
  Eigen::MatrixXd M(4 * obs.size(), 10);
  int row = 0;
  for (const ViewObservation& o : obs) {
    const Eigen::Vector4d bpx = bboxToPixels(o.bbox_norm, o.image_width, o.image_height);
    Eigen::Matrix<double, 3, 4> lines;
    lines.col(0) << 1, 0, -bpx(0);
    lines.col(1) << 1, 0, -bpx(2);
    lines.col(2) << 0, 1, -bpx(1);
    lines.col(3) << 0, 1, -bpx(3);
    for (int k = 0; k < 4; ++k) {
      const Eigen::Vector4d m = o.P.transpose() * lines.col(k);
      const Eigen::Matrix4d X = m * m.transpose();
      int c = 0;
      for (int r = 0; r < 4; ++r)
        for (int cc = r; cc < 4; ++cc) {
          double v = X(r, cc);
          if (r != cc) v *= std::sqrt(2.0);  // 保持 Frobenius 范数
          M(row, c++) = v;
        }
      ++row;
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
  const Eigen::VectorXd q10 = svd.matrixV().col(9);  // 最小奇异值对应向量

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  int c = 0;
  for (int r = 0; r < 4; ++r)
    for (int cc = r; cc < 4; ++cc) {
      double v = q10(c++);
      if (r != cc) v /= std::sqrt(2.0);
      Q(r, cc) = v;
      Q(cc, r) = v;
    }

  // 尺度规范 Q*(3,3) = -1
  if (std::abs(Q(3, 3)) < 1e-9) return fallback_cone();
  Q /= -Q(3, 3);

  // 施加椭球约束（docs/07 §4：把退化解投影到半正定锥得到正椭球）。
  // 对椭球，Q*[0:3,0:3] + c cᵀ 应半正定；若 SVD 解不满足，钳制半径重建。
  const Eigen::Vector3d center = -Q.block<3, 1>(0, 3);
  const Eigen::Matrix3d B = Q.block<3, 3>(0, 0) + center * center.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(B);
  if (es.eigenvalues()(0) <= 0.0) {
    const Eigen::Vector3d rr = es.eigenvalues().array().max(1e-6).sqrt();
    return DualQuadric::fromEllipsoid(center, rr, es.eigenvectors());
  }
  return DualQuadric(Q);
}

// =====================================================================
// docs/08 —— 物体级联合优化（g2o 因子图）
// =====================================================================
void QuadricOptimizer::optimize(
    ObjectMap& map, const std::vector<ViewObservation>& obs,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& pose_priors,
    std::vector<std::pair<int, Eigen::Isometry3d>>& refined_poses,
    const std::vector<std::pair<int, Eigen::Vector3d>>& object_points,
    const OptimizationContext& ctx) {
  refined_poses.clear();
  if (obs.empty() || pose_priors.empty()) return;

  // ---------- 建图 ----------
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 9>> BlockSolverT;
  auto linear_solver =
      std::make_unique<g2o::LinearSolverEigen<BlockSolverT::PoseMatrixType>>();
  auto block_solver = std::make_unique<BlockSolverT>(std::move(linear_solver));
  auto* algo = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
  optimizer.setAlgorithm(algo);

  // 1) 关键帧 SE3 顶点 + VINS 位姿先验边（固定首帧消除 gauge 自由度）
  std::map<int, g2o::VertexSE3*> pose_vx;
  bool first_pose = true;
  for (const auto& [fid, T] : pose_priors) {
    auto* v = new g2o::VertexSE3();
    v->setId(static_cast<int>(optimizer.vertices().size()));
    v->setEstimate(T);
    if (first_pose) {
      v->setFixed(true);
      first_pose = false;
    }
    optimizer.addVertex(v);
    pose_vx[fid] = v;

    auto* e = new EdgeSE3Prior();
    e->setVertex(0, v);
    e->setMeasurement(T);
    e->setInformation(ctx.pose_prior_weight *
                      Eigen::Matrix<double, 6, 6>::Identity());
    optimizer.addEdge(e);
  }

  // 2) 物体 quadric 顶点（初值取地图当前 Q*，规范到 (3,3)=-1）
  std::set<int> object_ids;
  for (const ViewObservation& o : obs) object_ids.insert(o.object_id);
  std::map<int, VertexQuadric*> quad_vx;
  for (int oid : object_ids) {
    const ObjectEntry* e = map.get(oid);
    if (e == nullptr) continue;
    Eigen::Matrix4d Q = e->quadric.matrix();
    if (std::abs(Q(3, 3)) < 1e-9) continue;
    Q /= -Q(3, 3);
    auto* v = new VertexQuadric();
    v->setId(static_cast<int>(optimizer.vertices().size()));
    v->setEstimate(Q);
    v->setMarginalized(true);  // 9 维 quadric 走 Schur 补 marginalize（对应 Hll 块）
    optimizer.addVertex(v);
    quad_vx[oid] = v;
  }

  // 3) bbox 约束边（docs/08 §3.1，核心约束）
  for (const ViewObservation& o : obs) {
    auto itq = quad_vx.find(o.object_id);
    if (itq == quad_vx.end()) continue;
    auto itp = pose_vx.find(o.frame_id);
    if (itp == pose_vx.end()) continue;
    auto* e = new EdgeQuadricBBox();
    e->setVertex(0, itq->second);
    e->setVertex(1, itp->second);
    e->setMeasurement(o.bbox_norm);
    e->K_ = ctx.K;
    e->image_width_ = ctx.image_width;
    e->image_height_ = ctx.image_height;
    e->setInformation(ctx.bbox_weight * Eigen::Matrix4d::Identity());
    optimizer.addEdge(e);
  }

  // 4) 点-物体约束边（docs/08 §3.3，可选）
  for (const auto& [oid, p] : object_points) {
    auto itq = quad_vx.find(oid);
    if (itq == quad_vx.end()) continue;
    auto* e = new EdgePointQuadric();
    e->setVertex(0, itq->second);
    e->p_ = p;
    e->setMeasurement(0.0);
    Eigen::Matrix<double, 1, 1> info;
    info(0, 0) = ctx.point_weight;
    e->setInformation(info);
    optimizer.addEdge(e);
  }

  if (optimizer.edges().empty()) return;

  // ---------- 求解（Levenberg-Marquardt） ----------
  optimizer.initializeOptimization();
  optimizer.optimize(ctx.max_iterations);

  // ---------- 回写：更新物体椭球 + 输出精化位姿 ----------
  for (const auto& [oid, v] : quad_vx) map.update(oid, DualQuadric(v->estimate()));
  refined_poses.reserve(pose_vx.size());
  for (const auto& [fid, v] : pose_vx) {
    refined_poses.emplace_back(fid, v->estimate());
  }
}

}  // namespace object_slam
