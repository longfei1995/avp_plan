#include "map/global_planner.h"

#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace avp {
namespace {
double LaneLength(const Lane& lane) {
  double result = 0.0;
  for (size_t i = 1; i < lane.centerline.size(); ++i) result += Distance(lane.centerline[i - 1], lane.centerline[i]);
  return result;
}
std::string ClosestLane(const std::vector<Lane>& lanes, const Vec2& position) {
  double best_distance = std::numeric_limits<double>::infinity();
  std::string result;
  for (const Lane& lane : lanes) {
    if (lane.closed) continue;
    for (const Vec2& point : lane.centerline) {
      const double distance = Distance(point, position);
      if (distance < best_distance) { best_distance = distance; result = lane.id; }
    }
  }
  return result;
}
}  // 匿名命名空间

bool GlobalPlanner::Plan(const PlanningFrame& frame, GlobalRoute* route, std::string* error) const {
  if (route == nullptr || error == nullptr || frame.map == nullptr) return false;
  const std::vector<Lane>& lanes = frame.map->lanes;
  const ParkingSpot* spot = nullptr;
  for (const ParkingSpot& candidate : frame.map->parking_spots) {
    if (candidate.id == frame.target_parking_spot_id) { spot = &candidate; break; }
  }
  if (spot == nullptr) { *error = "target parking spot not found"; return false; }
  const std::string start = ClosestLane(lanes, frame.ego.pose.position);
  const std::string goal = ClosestLane(lanes, spot->entry_pose.position);
  if (start.empty() || goal.empty()) { *error = "no open lane near start or parking entry"; return false; }
  std::unordered_map<std::string, const Lane*> lane_by_id;
  for (const Lane& lane : lanes) lane_by_id[lane.id] = &lane;
  const auto heuristic = [&lane_by_id, spot](const std::string& lane_id) {
    return Distance(lane_by_id.at(lane_id)->centerline.back(), spot->entry_pose.position);
  };
  struct QueueEntry {
    double priority;
    double g_cost;
    std::string id;
    bool operator>(const QueueEntry& other) const { return priority > other.priority; }
  };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  std::unordered_map<std::string, double> cost{{start, 0.0}};
  std::unordered_map<std::string, std::string> parent;
  open.push({heuristic(start), 0.0, start});
  while (!open.empty()) {
    const QueueEntry current = open.top(); open.pop();
    if (current.g_cost != cost[current.id]) continue;
    if (current.id == goal) break;
    const Lane* lane = lane_by_id[current.id];
    for (const std::string& successor : lane->successor_ids) {
      const auto found = lane_by_id.find(successor);
      if (found == lane_by_id.end() || found->second->closed) continue;
      const double next_cost = current.g_cost + LaneLength(*found->second);
      if (!cost.contains(successor) || next_cost < cost[successor]) {
        cost[successor] = next_cost;
        parent[successor] = current.id;
        open.push({next_cost + heuristic(successor), next_cost, successor});
      }
    }
  }
  if (!cost.contains(goal)) { *error = "parking entry is unreachable in lane graph"; return false; }
  std::vector<std::string> reversed;
  for (std::string lane_id = goal;; lane_id = parent[lane_id]) {
    reversed.push_back(lane_id);
    if (lane_id == start) break;
  }
  route->lane_ids.assign(reversed.rbegin(), reversed.rend());
  route->reference_line.clear();
  for (const std::string& lane_id : route->lane_ids) {
    const Lane* lane = lane_by_id[lane_id];
    for (const Vec2& point : lane->centerline) {
      if (route->reference_line.empty() || Distance(route->reference_line.back(), point) > 1e-4) route->reference_line.push_back(point);
    }
  }
  if (Distance(route->reference_line.back(), spot->entry_pose.position) > 0.1) {
    route->reference_line.push_back(spot->entry_pose.position);
  }
  const Vec2 previous = route->reference_line.size() > 1 ?
      route->reference_line[route->reference_line.size() - 2] : route->reference_line.back();
  const Pose2d entry{route->reference_line.back(),
                     std::atan2(route->reference_line.back().y - previous.y,
                                route->reference_line.back().x - previous.x)};
  std::vector<Pose2d> connection;
  if (!hybrid_a_star_.Plan(frame, entry, spot->target_pose, &connection, error)) return false;
  for (const Pose2d& point : connection) {
    if (Distance(route->reference_line.back(), point.position) > 0.1) {
      route->reference_line.push_back(point.position);
    }
  }
  route->parking_target = spot->target_pose;
  return true;
}
}  // avp 命名空间
