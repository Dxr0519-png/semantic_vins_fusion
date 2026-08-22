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

  // 从原始锥 C（齐次 3x3，任意尺度）提取 2D 椭圆：中心 center + 协方差 covariance。
  // 约定：椭圆轮廓恰为"单位马氏距离"等值线 (u-center)ᵀ Σ⁻¹ (u-center) = 1。
  // 返回 false：非椭圆（双曲线/抛物线/点/中心在无穷远/数值不健康），不修改输出。
  // 数学见 docs/07 §2.3 / docs/06 §3.4。
  static bool ellipseFromConic(const Eigen::Matrix3d& C,
                               Eigen::Vector2d& center,
                               Eigen::Matrix2d& covariance);

  // 投影得到 2D 椭圆（中心 + 协方差，像素系），供马氏距离门控（docs/06 §3.4）。
  // 内部先做 det/范数/finite 守卫再反演，退化投影（种子锥/奇异）返回 false，不产生 NaN。
  bool projectedEllipse(const Eigen::Matrix<double, 3, 4>& P,
                        Eigen::Vector2d& center_px,
                        Eigen::Matrix2d& covariance_px) const;

  // 是否为正椭球（非退化）：Q* 的 4 个特征值均非零（rel_eps 相对容差）。
  // 用于区分精化椭球与秩 3 退化种子锥（docs/07 §3 的 fromBBoxCone，恰有一个特征值=0）。
  bool isProper(double rel_eps = 1e-9) const;

  // 分解出中心 / 半轴 / 旋转（对正椭球 Q* 有效）
  void decompose(Eigen::Vector3d& center, Eigen::Vector3d& radii,
                 Eigen::Matrix3d& R) const;

  // 点相对椭球的归一化符号距离（<0 内部，>0 外部），用于点-物体关联
  double signedDistance(const Eigen::Vector3d& p) const;

private:
  Eigen::Matrix4d Q_star_;
};

}  // namespace object_slam
