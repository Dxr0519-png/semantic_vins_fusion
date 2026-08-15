#include "object_slam/object_map.h"

namespace object_slam {

ObjectEntry* ObjectMap::get(int id) {
  auto it = entries_.find(id);
  return it == entries_.end() ? nullptr : &it->second;
}

int ObjectMap::add(int class_id, float confidence, const DualQuadric& q) {
  ObjectEntry e;
  e.id = static_cast<int>(entries_.size());  // 简单递增，实际可复用已删除 id
  e.class_id = class_id;
  e.confidence = confidence;
  e.quadric = q;
  entries_[e.id] = e;
  return e.id;
}

void ObjectMap::update(int id, const DualQuadric& q) {
  auto it = entries_.find(id);
  if (it == entries_.end()) return;
  it->second.quadric = q;
}

void ObjectMap::erase(int id) { entries_.erase(id); }

void ObjectMap::markObserved(int id, int frame_id) {
  auto it = entries_.find(id);
  if (it == entries_.end()) return;
  it->second.observations += 1;
  it->second.last_seen = frame_id;
  it->second.valid = it->second.observations >= /*min_observations*/ 3;
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
