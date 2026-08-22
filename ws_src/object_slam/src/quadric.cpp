#include "object_slam/quadric.h"

#include <algorithm>
#include <cmath>

namespace object_slam {

DualQuadric::DualQuadric() {
  Q_star_ = Eigen::Matrix4d::Identity();
  Q_star_(3, 3) = -1.0;  // 单位球
}

DualQuadric::DualQuadric(const Eigen::Matrix4d& Q_star) : Q_star_(Q_star) {}

DualQuadric DualQuadric::fromEllipsoid(const Eigen::Vector3d& center,
                                       const Eigen::Vector3d& radii,
                                       const Eigen::Matrix3d& R) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = center;

  Eigen::Matrix4d D = Eigen::Matrix4d::Zero();
  D(0, 0) = radii.x() * radii.x();
  D(1, 1) = radii.y() * radii.y();
  D(2, 2) = radii.z() * radii.z();
  D(3, 3) = -1.0;

  return DualQuadric(T * D * T.transpose());
}

DualQuadric DualQuadric::fromBBoxCone(const Eigen::Vector4d& bbox_px,
                                      const Eigen::Matrix<double, 3, 4>& P) {
  const double x1 = bbox_px(0), y1 = bbox_px(1);
  const double x2 = bbox_px(2), y2 = bbox_px(3);

  // 四条 bbox 边线（齐次直线）：x=x1, x=x2, y=y1, y=y2
  Eigen::Vector3d l_left(1.0, 0.0, -x1);
  Eigen::Vector3d l_right(1.0, 0.0, -x2);
  Eigen::Vector3d l_top(0.0, 1.0, -y1);
  Eigen::Vector3d l_bottom(0.0, 1.0, -y2);

  // 对偶锥 C*：与四条边相切（Cross & Zisserman）
  Eigen::Matrix3d Cs = l_left * l_right.transpose() + l_right * l_left.transpose() +
                       l_top * l_bottom.transpose() + l_bottom * l_top.transpose();

  // 反投影到世界：Q* = Pᵀ C* P（锥顶点在相机光心，秩 3）
  Eigen::Matrix4d Q = P.transpose() * Cs * P;
  return DualQuadric(Q);
}

Eigen::Matrix3d DualQuadric::projectDual(const Eigen::Matrix<double, 3, 4>& P) const {
  return P * Q_star_ * P.transpose();
}

Eigen::Matrix3d DualQuadric::projectPrimal(const Eigen::Matrix<double, 3, 4>& P) const {
  Eigen::Matrix3d Cs = projectDual(P);
  // 归一化：避免大尺度导致数值问题
  double n = Cs.norm();
  if (n < 1e-12) n = 1.0;
  Cs /= n;
  // 原始锥 = 对偶锥的逆（用伴随矩阵更稳健）
  Eigen::Matrix3d C = Cs.inverse();
  if (std::abs(C(2, 2)) > 1e-12) C /= C(2, 2);  // 齐次归一化
  return C;
}

bool DualQuadric::ellipseFromConic(const Eigen::Matrix3d& C,
                                   Eigen::Vector2d& center,
                                   Eigen::Matrix2d& covariance) {
  // 数学（docs/07 §2.3 / docs/06 §3.4）：椭圆 (u-c)ᵀ Σ⁻¹ (u-c) = 1 的齐次锥为
  //   C = [ Σ⁻¹  -Σ⁻¹c； -(Σ⁻¹c)ᵀ  cᵀΣ⁻¹c - 1 ]（任意尺度）
  // 中心 = 无穷远线 l∞=[0,0,1]ᵀ 关于 C 的极点：ch = C⁻¹ l∞。
  if (!C.allFinite()) return false;
  const Eigen::Matrix3d Cinv = C.inverse();
  if (!Cinv.allFinite()) return false;
  const Eigen::Vector3d ch = Cinv * Eigen::Vector3d(0.0, 0.0, 1.0);
  if (std::abs(ch(2)) < 1e-12) return false;  // 中心在无穷远（抛物线），非椭圆
  center = ch.head<2>() / ch(2);

  // 平移至中心后二次部分 Q 与中心处常数值 f0：Σ⁻¹ = -Q/f0（尺度无关，已验证）。
  // 半轴平方 sᵢ² = -f0/λᵢ，λᵢ 为 Q 的特征值。
  const Eigen::Matrix2d Q = C.block<2, 2>(0, 0);
  const Eigen::Vector3d h(center.x(), center.y(), 1.0);
  const double f0 = h.transpose() * C * h;
  if (std::abs(f0) < 1e-12) return false;  // 过中心的退化锥（点 / 两直线）

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(Q);
  if (es.info() != Eigen::Success) return false;
  const Eigen::Vector2d lam = es.eigenvalues();
  const Eigen::Matrix2d R = es.eigenvectors();
  // 对椭圆要求 -f0/λᵢ > 0（Q 两特征值同号且与 f0 异号）；双曲线会给出负半轴。
  const Eigen::Vector2d s2 = (-f0 / lam.array()).matrix();
  if (!s2.allFinite() || s2.minCoeff() <= 1e-12) return false;
  covariance = R * s2.asDiagonal() * R.transpose();
  return true;
}

bool DualQuadric::projectedEllipse(const Eigen::Matrix<double, 3, 4>& P,
                                   Eigen::Vector2d& center_px,
                                   Eigen::Matrix2d& covariance_px) const {
  // 对偶锥 C* = P Q* Pᵀ（像素系，3x3 对称）
  Eigen::Matrix3d Cs = P * Q_star_ * P.transpose();
  if (!Cs.allFinite()) return false;

  // 守卫：奇异投影不反演（秩 3 种子锥 / 相机退化），避免 NaN 传播
  const double d = Cs.determinant();
  if (!std::isfinite(d) || std::abs(d) < 1e-12) return false;

  // 数值稳定归一化（同 projectPrimal），再求原始锥 C = C*⁻¹
  double n = Cs.norm();
  if (n < 1e-12) return false;
  Cs /= n;
  return ellipseFromConic(Cs.inverse(), center_px, covariance_px);
}

bool DualQuadric::isProper(double rel_eps) const {
  // 正椭球 Q* 非奇异（特征值 rx²,ry²,rz²,-1 全非零）；秩 3 种子锥恰有一个特征值=0。
  if (!Q_star_.allFinite()) return false;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(Q_star_);
  if (es.info() != Eigen::Success) return false;
  const Eigen::Vector4d ev = es.eigenvalues();
  const double lmax = ev.cwiseAbs().maxCoeff();
  if (!(lmax > 0.0) || !std::isfinite(lmax)) return false;
  for (int i = 0; i < 4; ++i) {
    if (std::abs(ev(i)) <= rel_eps * lmax) return false;
  }
  return true;
}

void DualQuadric::decompose(Eigen::Vector3d& center, Eigen::Vector3d& radii,
                            Eigen::Matrix3d& R) const {
  // Q* = [R S Rᵀ - c cᵀ,  -c;  -cᵀ,  -1]（相差一个尺度）
  // 1) 归一化使 Q*(3,3) = -1
  double s = -Q_star_(3, 3);
  if (std::abs(s) < 1e-12) s = 1.0;
  Eigen::Matrix4d Q = Q_star_ / s;

  // 2) 中心
  center = -Q.block<3, 1>(0, 3);

  // 3) R S Rᵀ = A + c cᵀ
  Eigen::Matrix3d A = Q.block<3, 3>(0, 0);
  Eigen::Matrix3d B = A + center * center.transpose();

  // 4) 特征分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(B);
  R = es.eigenvectors();
  Eigen::Vector3d eig = es.eigenvalues();
  radii = eig.array().max(1e-6).sqrt();
}

double DualQuadric::signedDistance(const Eigen::Vector3d& p) const {
  Eigen::Vector3d center, radii;
  Eigen::Matrix3d R;
  decompose(center, radii, R);
  Eigen::Vector3d q = R.transpose() * (p - center);
  Eigen::Vector3d n = q.array() / radii.array();  // 归一化坐标
  return n.norm() - 1.0;  // <0 内部，>0 外部（马氏距离近似）
}

}  // namespace object_slam
