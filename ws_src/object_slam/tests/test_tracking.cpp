// docs/06 物体级数据关联与跟踪 —— 单元验证。
// 覆盖：bboxIoU、matchingCost、associate(类别/IoU/马氏门控)、id 复用、
//       markObserved/prune 生命周期、Tracker 跨帧 id 稳定 + 遮挡恢复。
//
// 纯逻辑测试，不依赖 ROS。构建运行见 docs/06 §7：
//   colcon build --packages-select object_slam
//   ./install/object_slam/lib/object_slam/test_tracking
//
// 注意：用 CHECK（而非 assert）—— Release 构建下 NDEBUG 会让 assert 失效。

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

#include <Eigen/Geometry>

#include "object_slam/data_association.h"
#include "object_slam/object_map.h"
#include "object_slam/tracker.h"

using object_slam::DataAssociation;
using object_slam::Detection2DView;
using object_slam::DualQuadric;
using object_slam::ObjectMap;
using object_slam::TrackView;
using object_slam::Tracker;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::cerr << "CHECK failed: " #cond "  (test_tracking.cpp:" << __LINE__ \
                << ")\n";                                                    \
      std::exit(1);                                                          \
    }                                                                        \
  } while (0)

namespace {

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

Detection2DView makeDet(int class_id, double x1, double y1, double x2, double y2) {
  Detection2DView d;
  d.class_id = class_id;
  d.score = 0.9f;
  d.bbox = Eigen::Vector4d(x1, y1, x2, y2);
  return d;
}

TrackView makeTrack(int id, int class_id, double x1, double y1, double x2, double y2) {
  TrackView t;
  t.track_id = id;
  t.class_id = class_id;
  t.bbox = Eigen::Vector4d(x1, y1, x2, y2);
  // t.quadric 由结构体默认初始化为零矩阵 -> 马氏门控不激活（docs/06 §3.4）
  return t;
}

// 马氏门控测试场景（docs/06 §3.4）：相机内参 K、位姿 T_wc、投影矩阵 P、世界系椭球。
// 数字经核算：椭球中心 (0,0,3) 投影到 (320,240) px，投影椭圆 Σ=diag(1604.01, 712.89) px²，
// 半轴 (40.05, 26.70) px。
static void setupGateScene(Eigen::Matrix3d& K, Eigen::Isometry3d& T_wc,
                           Eigen::Matrix<double, 3, 4>& P,
                           DualQuadric& ellipsoid) {
  K << 400, 0, 320,
       0, 400, 240,
       0, 0, 1;
  T_wc = Eigen::Isometry3d::Identity();
  T_wc.linear() << 1, 0, 0,
                   0, -1, 0,
                   0, 0, -1;
  T_wc.translation() << 0, 0, 6;
  P = K * T_wc.matrix().block<3, 4>(0, 0);
  ellipsoid = DualQuadric::fromEllipsoid(Eigen::Vector3d(0, 0, 3),
                                         Eigen::Vector3d(0.3, 0.2, 0.15));
}

std::set<int> trackIds(const ObjectMap& m) {
  std::set<int> ids;
  for (const auto& [id, e] : m.entries()) ids.insert(id);
  return ids;
}

// 返回 map 中类别为 class_id 的物体 id（不存在返回 -1）
int findIdOfClass(const ObjectMap& m, int class_id) {
  for (const auto& [id, e] : m.entries())
    if (e.class_id == class_id) return id;
  return -1;
}

// ============================ §3.2 bboxIoU ============================
void testBboxIoU() {
  CHECK(near(DataAssociation::bboxIoU({0, 0, 1, 1}, {0, 0, 1, 1}), 1.0));
  CHECK(near(DataAssociation::bboxIoU({0, 0, 1, 1}, {0, 0, 0.5, 0.5}), 0.25));
  CHECK(near(DataAssociation::bboxIoU({0, 0, 1, 1}, {2, 2, 3, 3}), 0.0));
  // 部分重叠：A 与右移 0.5 的单位框，交集 0.5x1，并集 1.5 -> 1/3
  CHECK(near(DataAssociation::bboxIoU({0, 0, 1, 1}, {0.5, 0, 1.5, 1}), 1.0 / 3.0));
  std::cout << "  [ok] bboxIoU\n";
}

// ======================== §3.1/§3.2 matchingCost ======================
void testMatchingCost() {
  const Detection2DView d = makeDet(1, 0, 0, 1, 1);
  TrackView t = makeTrack(5, 1, 0, 0, 1, 1);
  CHECK(near(DataAssociation::matchingCost(d, t), 0.0));  // 同类别同框 -> 0
  t.bbox = Eigen::Vector4d(0, 0, 0.5, 0.5);
  CHECK(near(DataAssociation::matchingCost(d, t), 0.75)); // 1 - IoU
  t.class_id = 2;                                         // 类别不一致 -> inf
  CHECK(std::isinf(DataAssociation::matchingCost(d, t)));
  std::cout << "  [ok] matchingCost\n";
}

// ========================== §4 associate 门控 ==========================
void testAssociate() {
  DataAssociation da;  // 默认 IoU 阈值 0.3
  // 存量用例 track 的 quadric 默认是零矩阵（无有效椭球）-> 马氏门控不激活，P 不被使用
  const Eigen::Matrix<double, 3, 4> Pid = Eigen::Matrix<double, 3, 4>::Identity();
  const Detection2DView d_a = makeDet(0, 0, 0, 0.3, 0.3);
  const Detection2DView d_b = makeDet(1, 0.6, 0.6, 0.9, 0.9);

  // 正常匹配：两个 track 各找到同类别、高 IoU 的检测
  {
    const std::vector<TrackView> tracks = {makeTrack(0, 0, 0, 0, 0.3, 0.3),
                                           makeTrack(1, 1, 0.6, 0.6, 0.9, 0.9)};
    const auto a = da.associate({d_a, d_b}, tracks, Pid, 640, 480);
    CHECK(a.size() == 2u);
    CHECK(a[0].track_id == 0 && a[0].detection_index == 0);
    CHECK(a[1].track_id == 1 && a[1].detection_index == 1);
  }

  // 类别不一致 -> 无匹配
  {
    const std::vector<TrackView> tracks = {makeTrack(0, 2, 0, 0, 0.3, 0.3)};
    CHECK(da.associate({d_a}, tracks, Pid, 640, 480).empty());
  }

  // IoU 低于阈值(0.3) -> 无匹配：检测 {0.8,0,1.8,1} 与 track {0,0,1,1} IoU≈0.111
  {
    const TrackView t = makeTrack(0, 0, 0, 0, 1, 1);
    const Detection2DView d = makeDet(0, 0.8, 0, 1.8, 1);
    CHECK(da.associate({d}, {t}, Pid, 640, 480).empty());
  }

  // 调低阈值后同上场景 -> 匹配成功
  {
    DataAssociation loose;
    loose.setIoUThreshold(0.05);
    const TrackView t = makeTrack(0, 0, 0, 0, 1, 1);
    const Detection2DView d = makeDet(0, 0.8, 0, 1.8, 1);
    const auto a = loose.associate({d}, {t}, Pid, 640, 480);
    CHECK(a.size() == 1u && a[0].track_id == 0);
  }

  // 贪心：一个检测被占用后，另一 track 不能抢到低重叠检测
  {
    const std::vector<TrackView> tracks = {makeTrack(0, 0, 0, 0, 1, 1),
                                           makeTrack(1, 0, 0, 0, 0.2, 0.2)};
    const auto a = da.associate({makeDet(0, 0, 0, 1, 1)}, tracks, Pid, 640, 480);
    CHECK(a.size() == 1u && a[0].track_id == 0);  // 只有 track0 匹配成功
  }
  std::cout << "  [ok] associate (门控/类别/贪心)\n";
}

// ================== docs/06 §3.4 锥 -> 椭圆提取（纯数学） ==================
void testEllipseFromConic() {
  // 已知椭圆：(u-100)²/400 + (v-50)²/100 = 1，即 center=(100,50)，Σ=diag(400,100)。
  // 齐次锥 C（任意尺度，C(2,2) 未归一化）：Σ⁻¹=diag(1/400,1/100)，cᵀΣ⁻¹c-1=49。
  Eigen::Matrix3d C;
  C << 1.0 / 400, 0.0, -(100.0 / 400),
       0.0, 1.0 / 100, -(50.0 / 100),
       -(100.0 / 400), -(50.0 / 100), 49.0;
  Eigen::Vector2d center;
  Eigen::Matrix2d cov;
  CHECK(DualQuadric::ellipseFromConic(C, center, cov));
  CHECK(near(center.x(), 100.0, 1e-6));
  CHECK(near(center.y(), 50.0, 1e-6));
  CHECK(near(cov(0, 0), 400.0, 1e-6));
  CHECK(near(cov(1, 1), 100.0, 1e-6));
  CHECK(near(cov(0, 1), 0.0, 1e-6));
  // 单位马氏性质：边界点 (120,50) 上 (u-μ)ᵀΣ⁻¹(u-μ) = 1
  const Eigen::Vector2d d(120.0 - 100.0, 50.0 - 50.0);
  CHECK(near(d.dot(cov.inverse() * d), 1.0, 1e-6));

  // 非椭圆被拒绝：双曲线 / 抛物线（中心在无穷远）/ 虚椭圆 / 点退化
  Eigen::Matrix3d hyp; hyp << 1, 0, 0,  0, -1, 0,  0, 0, -1;
  CHECK(!DualQuadric::ellipseFromConic(hyp, center, cov));
  Eigen::Matrix3d par; par << 1, 0, 0,  0, 0, -0.5,  0, -0.5, 0;
  CHECK(!DualQuadric::ellipseFromConic(par, center, cov));
  Eigen::Matrix3d imag; imag << 1, 0, 0,  0, 1, 0,  0, 0, 1;
  CHECK(!DualQuadric::ellipseFromConic(imag, center, cov));
  std::cout << "  [ok] ellipseFromConic (中心/Σ/单位马氏/退化拒绝)\n";
}

// ================= docs/06 §3.4 associate 马氏距离门控 =================
void testMahalanobisGate() {
  Eigen::Matrix3d K;
  Eigen::Isometry3d T_wc;
  Eigen::Matrix<double, 3, 4> P;
  DualQuadric ellipsoid;
  setupGateScene(K, T_wc, P, ellipsoid);

  DataAssociation da;  // 默认 IoU 0.3、马氏 chi2 5.991

  // 自检：椭球为正椭球；投影椭圆中心 (320,240)、Σ 对角元符合核算值
  CHECK(ellipsoid.isProper());
  Eigen::Vector2d ell_c;
  Eigen::Matrix2d sigma_px;
  CHECK(ellipsoid.projectedEllipse(P, ell_c, sigma_px));
  CHECK(near(ell_c.x(), 320.0, 1e-6));
  CHECK(near(ell_c.y(), 240.0, 1e-6));
  CHECK(near(sigma_px(0, 0), 1604.01, 1e-1));
  CHECK(near(sigma_px(1, 1), 712.89, 1e-1));

  // B1 同物通过：检测 bbox 中心 = (0.5,0.5) = 投影中心 -> d²≈0，IoU=0.643 >= 0.3
  {
    TrackView t = makeTrack(0, 0, 0.43, 0.42, 0.57, 0.58);
    t.quadric = ellipsoid.matrix();
    const Detection2DView d = makeDet(0, 0.44, 0.44, 0.56, 0.56);
    const auto a = da.associate({d}, {t}, P, 640, 480);
    CHECK(a.size() == 1u && a[0].track_id == 0);
  }

  // B2 马氏拒绝（关键判别）：track 框全图（IoU=0.33 仍 >= 0.3 通过 IoU 门控），
  //    但检测中心 (384,156) 相对投影中心 (320,240) 的 d²=12.45 >= 5.991 -> 无匹配。
  //    调大阈值 / 关闭门控后翻转 -> 证明是马氏在拒绝而非 IoU。
  {
    TrackView t = makeTrack(0, 0, 0.00, 0.00, 1.00, 1.00);
    t.quadric = ellipsoid.matrix();
    const Detection2DView d = makeDet(0, 0.30, 0.05, 0.90, 0.60);
    CHECK(da.associate({d}, {t}, P, 640, 480).empty());

    DataAssociation loose;
    loose.setChi2Threshold(100.0);
    CHECK(loose.associate({d}, {t}, P, 640, 480).size() == 1u);

    DataAssociation off;
    off.setChi2Threshold(0.0);  // 关闭门控 -> 纯 IoU
    CHECK(off.associate({d}, {t}, P, 640, 480).size() == 1u);
  }

  // B3 零矩阵 quadric（无有效椭球）-> 跳过马氏门控，远检测按 IoU 匹配成功
  {
    const TrackView t = makeTrack(0, 0, 0.00, 0.00, 1.00, 1.00);
    const Detection2DView d = makeDet(0, 0.30, 0.05, 0.90, 0.60);
    CHECK(da.associate({d}, {t}, P, 640, 480).size() == 1u);
  }

  // B3' 种子锥 quadric（秩 3 退化）-> isProper()=false -> 跳过马氏门控
  {
    const Eigen::Vector4d bbox_px(0.44 * 640, 0.44 * 480, 0.56 * 640, 0.56 * 480);
    const DualQuadric cone = DualQuadric::fromBBoxCone(bbox_px, P);
    CHECK(!cone.isProper());
    TrackView t = makeTrack(0, 0, 0.00, 0.00, 1.00, 1.00);
    t.quadric = cone.matrix();
    const Detection2DView d = makeDet(0, 0.30, 0.05, 0.90, 0.60);
    CHECK(da.associate({d}, {t}, P, 640, 480).size() == 1u);
  }
  std::cout << "  [ok] associate 马氏距离门控（docs/06 §3.4）\n";
}

// ==================== §6 id 复用 + §5 生命周期原语 ====================
void testObjectMap() {
  ObjectMap m;
  const int id_a = m.add(0, 0.9f, DualQuadric());
  const int id_b = m.add(1, 0.8f, DualQuadric());
  const int id_c = m.add(2, 0.7f, DualQuadric());
  CHECK(id_a != id_b && id_b != id_c);

  // 旧 bug 场景：erase 中间一个 id 后，entries_.size() 会与现存 id 冲突。
  // 新实现从空闲列表复用被删 id，绝不与现存 id 冲突。
  m.erase(id_a);                       // 回收 id_a
  const int id_d = m.add(3, 0.6f, DualQuadric());
  CHECK(id_d == id_a);                 // id 复用（docs/06 §6）
  CHECK(m.get(id_b) && m.get(id_c));   // 其余物体完好
  CHECK(trackIds(m).size() == 3u);     // 3 个不重复 key
  std::cout << "  [ok] ObjectMap::add id 复用（无冲突）\n";

  // markObserved：观测达到 min_observations 才 valid
  {
    ObjectMap m2;
    const int id = m2.add(0, 0.9f, DualQuadric());
    m2.markObserved(id, 0, 3);
    CHECK(!m2.get(id)->valid);
    m2.markObserved(id, 1, 3);
    CHECK(!m2.get(id)->valid);
    m2.markObserved(id, 2, 3);
    CHECK(m2.get(id)->valid);
  }
  std::cout << "  [ok] markObserved (min_observations 门控)\n";

  // prune：超过 max_miss 帧未观测删除，否则保留
  {
    ObjectMap m3;
    const int id = m3.add(0, 0.9f, DualQuadric());
    m3.markObserved(id, 10, 3);
    m3.prune(10, 30);          // 10-10=0 -> 保留
    CHECK(m3.get(id) != nullptr);
    m3.prune(100, 30);         // 100-10=90 > 30 -> 删除
    CHECK(m3.get(id) == nullptr);
  }
  std::cout << "  [ok] prune (短暂遮挡容忍 / 超期消亡)\n";
}

// ============== Tracker：跨帧 id 稳定 + 遮挡恢复（docs/06 目标） =========
void testTrackerStableId() {
  Tracker tr(Eigen::Matrix3d::Identity(), 640, 480);
  tr.setMinObservations(1);  // 让测试聚焦 id 稳定而非观测门控

  const Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // 帧0：两个物体 A(class0)、B(class1)
  tr.updateFrame({makeDet(0, 0.00, 0.00, 0.30, 0.30),
                  makeDet(1, 0.60, 0.60, 0.90, 0.90)}, T);
  const auto ids0 = trackIds(tr.map());
  CHECK(ids0.size() == 2u);
  const int b_id = findIdOfClass(tr.map(), 1);
  CHECK(b_id != -1);

  // 帧1：两个物体微移 -> 同一 id，不新建
  tr.updateFrame({makeDet(0, 0.02, 0.02, 0.32, 0.32),
                  makeDet(1, 0.62, 0.62, 0.92, 0.92)}, T);
  CHECK(trackIds(tr.map()) == ids0);
  std::cout << "  [ok] Tracker 跨帧 id 稳定\n";

  // 遮挡：A 消失两帧 -> 短暂遮挡不删除（max_missed_frames 内保留原 id）
  tr.updateFrame({makeDet(1, 0.63, 0.63, 0.93, 0.93)}, T);
  tr.updateFrame({makeDet(1, 0.63, 0.63, 0.93, 0.93)}, T);
  CHECK(trackIds(tr.map()) == ids0);  // A 仍在（miss<30）
  std::cout << "  [ok] Tracker 短暂遮挡不删（保留 id）\n";

  // 遮挡恢复：A 重新出现 -> 匹配回原 id，不新建
  tr.updateFrame({makeDet(0, 0.00, 0.00, 0.30, 0.30),
                  makeDet(1, 0.63, 0.63, 0.93, 0.93)}, T);
  CHECK(trackIds(tr.map()) == ids0);
  std::cout << "  [ok] Tracker 遮挡恢复回原 id\n";

  // 长期消失：A 连续 40 帧不再出现 -> 超过 max_missed_frames 后被删除
  for (int i = 0; i < 40; ++i) {
    tr.updateFrame({makeDet(1, 0.63, 0.63, 0.93, 0.93)}, T);
  }
  CHECK(trackIds(tr.map()).size() == 1u);      // 只剩 B
  CHECK(findIdOfClass(tr.map(), 1) == b_id);   // B 的 id 从头到尾不变
  CHECK(findIdOfClass(tr.map(), 0) == -1);     // A 已消亡
  std::cout << "  [ok] Tracker 长期消失 -> 消亡\n";

  // IoU 门控防止串 id：同类别新物体离 A 很远时不能抢 A 的 id
  tr.updateFrame({makeDet(1, 0.63, 0.63, 0.93, 0.93),
                  makeDet(0, 0.80, 0.05, 0.90, 0.15)}, T);  // 新 C 与 A 无重叠
  CHECK(trackIds(tr.map()).size() == 2u);  // B + C，各自独立 id
  std::cout << "  [ok] Tracker IoU 门控防串 id\n";
}

}  // namespace

int main() {
  std::cout << "docs/06 跟踪验证开始...\n";
  testBboxIoU();
  testMatchingCost();
  testAssociate();
  testEllipseFromConic();
  testMahalanobisGate();
  testObjectMap();
  testTrackerStableId();
  std::cout << "全部通过 ✓\n";
  return 0;
}
