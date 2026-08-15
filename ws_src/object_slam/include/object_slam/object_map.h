#pragma once

#include <map>

#include "object_slam/quadric.h"

namespace object_slam {

// 地图中的一个物体
struct ObjectEntry {
  int id = -1;
  int class_id = -1;
  float confidence = 0.0f;
  DualQuadric quadric;   // 对偶二次曲面（世界系）
  int observations = 0;  // 累计观测次数
  int last_seen = 0;     // 最近一次被观测到的帧号
  bool valid = false;    // 观测足够、可输出的标志
};

// 语义物体地图容器（见 docs/06、07）
class ObjectMap {
public:
  ObjectEntry* get(int id);
  const std::map<int, ObjectEntry>& entries() const { return entries_; }

  // 新建物体，返回分配的 id
  int add(int class_id, float confidence, const DualQuadric& q);

  // 用新观测更新椭球
  void update(int id, const DualQuadric& q);

  void erase(int id);

  // 记录一次观测（用于跟踪生命周期）
  void markObserved(int id, int frame_id);

  // 清理超过 max_miss 帧未观测的物体
  void prune(int current_frame, int max_miss);

private:
  std::map<int, ObjectEntry> entries_;
};

}  // namespace object_slam
