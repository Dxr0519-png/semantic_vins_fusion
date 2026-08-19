#pragma once

#include <map>
#include <set>

#include <Eigen/Core>

#include "object_slam/quadric.h"

namespace object_slam {

// 地图中的一个物体
struct ObjectEntry {
  int id = -1;
  int class_id = -1;
  float confidence = 0.0f;
  DualQuadric quadric;           // 对偶二次曲面（世界系）
  Eigen::Vector4d bbox_smoothed; // 最近一次观测的 EMA 平滑 bbox（归一化），供帧间关联
  int observations = 0;          // 累计观测次数
  int last_seen = 0;             // 最近一次被观测到的帧号
  bool valid = false;            // 观测足够、可输出的标志
};

// 语义物体地图容器（见 docs/06、07）
class ObjectMap {
public:
  ObjectEntry* get(int id);
  const std::map<int, ObjectEntry>& entries() const { return entries_; }

  // 新建物体，返回分配的 id。删除的 id 会被回收复用（docs/06 §6）。
  int add(int class_id, float confidence, const DualQuadric& q);

  // 用新观测更新椭球
  void update(int id, const DualQuadric& q);

  // 删除物体并把 id 放回空闲列表供复用
  void erase(int id);

  // 记录一次观测（用于跟踪生命周期）；观测达到 min_observations 才置 valid
  void markObserved(int id, int frame_id, int min_observations);

  // 清理超过 max_miss 帧未观测的物体
  void prune(int current_frame, int max_miss);

private:
  std::map<int, ObjectEntry> entries_;
  std::set<int> free_ids_;  // 已删除、可复用的 id
  int next_id_ = 0;         // 单调递增，从未复用过的 id
};

}  // namespace object_slam
