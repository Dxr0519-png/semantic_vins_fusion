#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace object_slam {

// 对偶二次曲面（椭球）Q*：4x4 对称矩阵，9 自由度。
// 定义：平面 π 与椭球相切  <=>  πᵀ Q* π = 0。
// 数学与推导见 docs/07。
class DualQuadric {
public:
  DualQuadric();                                  // 默认：单位球
  explicit DualQuadric(const Eigen::Matrix4d& Q_star);

  // 由中心 + 三轴半径 + 旋转构造：
  //   Q* = T diag(rx², ry², rz², -1) Tᵀ,  T = [R, c; 0, 1]
  static DualQuadric fromEllipsoid(const Eigen::Vector3d& center,
                                   const Eigen::Vector3d& radii,
                                   const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity());

  // 从单帧 bbox 反投影锥初始化（退化的秩 3 锥，需多视图精化，见 docs/07 §3）
  // bbox_px: 像素坐标 [xmin, ymin, xmax, ymax]
  // P:        完整投影矩阵 K[R|t]（世界 -> 像素，3x4）
  static DualQuadric fromBBoxCone(const Eigen::Vector4d& bbox_px,
                                  const Eigen::Matrix<double, 3, 4>& P);

  const Eigen::Matrix4d& matrix() const { return Q_star_; }

  // 投影到图像：对偶锥 C* = P Q* Pᵀ（3x3 对称）
  Eigen::Matrix3d projectDual(const Eigen::Matrix<double, 3, 4>& P) const;

  // 投影得到的椭圆（原始锥 C = C*^{-1}，齐次归一化）
  Eigen::Matrix3d projectPrimal(const Eigen::Matrix<double, 3, 4>& P) const;

  // 分解出中心 / 半轴 / 旋转（对正椭球 Q* 有效）
  void decompose(Eigen::Vector3d& center, Eigen::Vector3d& radii,
                 Eigen::Matrix3d& R) const;

  // 点相对椭球的归一化符号距离（<0 内部，>0 外部），用于点-物体关联
  double signedDistance(const Eigen::Vector3d& p) const;

private:
  Eigen::Matrix4d Q_star_;
};

}  // namespace object_slam
