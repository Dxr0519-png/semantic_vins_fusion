#include "object_slam/object_map.h"

namespace object_slam {

ObjectEntry* ObjectMap::get(int id) {
  auto it = entries_.find(id);
  return it == entries_.end() ? nullptr : &it->second;
}

int ObjectMap::add(int class_id, float confidence, const DualQuadric& q) {
  ObjectEntry e;
  // id 分配：优先复用已删除的 id，否则分配新的单调递增 id（docs/06 §6 id 复用）。
  // 不能用 entries_.size()——删除后 size 会与现存 id 冲突，覆盖已有物体。
  int id;
  if (!free_ids_.empty()) {
    id = *free_ids_.begin();
    free_ids_.erase(free_ids_.begin());
  } else {
    id = next_id_++;
  }
  e.id = id;
  e.class_id = class_id;
  e.confidence = confidence;
  e.quadric = q;
  entries_[id] = e;
  return id;
}

void ObjectMap::update(int id, const DualQuadric& q) {
  auto it = entries_.find(id);
  if (it == entries_.end()) return;
  it->second.quadric = q;
}

void ObjectMap::erase(int id) {
  if (entries_.erase(id) > 0) free_ids_.insert(id);  // 回收 id 供复用
}

void ObjectMap::markObserved(int id, int frame_id, int min_observations) {
  auto it = entries_.find(id);
  if (it == entries_.end()) return;
  it->second.observations += 1;
  it->second.last_seen = frame_id;
  it->second.valid = it->second.observations >= min_observations;
}

void ObjectMap::prune(int current_frame, int max_miss) {
  for (auto it = entries_.begin(); it != entries_.end();) {
    if (current_frame - it->second.last_seen > max_miss) {
      it = entries_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace object_slam
