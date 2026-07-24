#include "map/global_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

namespace avp {
namespace {
constexpr double kGeometryEpsilon = 1e-6;
constexpr double kAppendEpsilon = 1e-4;

struct LaneProjection {
  const Lane* lane = nullptr;
  size_t segment_index = 0;
  double segment_ratio = 0.0;
  Vec2 position;
  double s = 0.0;
  double yaw = 0.0;
  double distance = std::numeric_limits<double>::infinity();
};

double LaneLength(const Lane& lane) {
  double result = 0.0;
  for (size_t i = 1; i < lane.centerline.size(); ++i) {
    result += Distance(lane.centerline[i - 1], lane.centerline[i]);
  }
  return result;
}

LaneProjection LaneStart(const Lane& lane) {
  const Vec2& first = lane.centerline.front();
  const Vec2& second = lane.centerline[1];
  return {&lane, 0, 0.0, first, 0.0,
          std::atan2(second.y - first.y, second.x - first.x), 0.0};
}

LaneProjection LaneEnd(const Lane& lane) {
  const size_t segment_index = lane.centerline.size() - 2;
  const Vec2& first = lane.centerline[segment_index];
  const Vec2& last = lane.centerline.back();
  return {&lane, segment_index, 1.0, last, LaneLength(lane),
          std::atan2(last.y - first.y, last.x - first.x), 0.0};
}

std::optional<LaneProjection> ProjectOntoLane(const Lane& lane, const Vec2& position) {
  std::optional<LaneProjection> best;
  double s_before_segment = 0.0;
  for (size_t index = 1; index < lane.centerline.size(); ++index) {
    const Vec2& start = lane.centerline[index - 1];
    const Vec2& end = lane.centerline[index];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < kGeometryEpsilon * kGeometryEpsilon) {
      continue;
    }
    const double segment_length = std::sqrt(squared_length);
    const double unclamped_ratio =
        ((position.x - start.x) * dx + (position.y - start.y) * dy) / squared_length;
    const double ratio = std::clamp(unclamped_ratio, 0.0, 1.0);
    const Vec2 projection{start.x + ratio * dx, start.y + ratio * dy};
    const double distance = Distance(position, projection);
    if (!best.has_value() || distance < best->distance) {
      best = LaneProjection{&lane,
                            index - 1,
                            ratio,
                            projection,
                            s_before_segment + ratio * segment_length,
                            std::atan2(dy, dx),
                            distance};
    }
    s_before_segment += segment_length;
  }
  return best;
}

std::optional<LaneProjection> ClosestProjection(const std::vector<Lane>& lanes,
                                                 const Vec2& position,
                                                 const PlannerConfig& config,
                                                 std::optional<double> required_yaw) {
  std::optional<LaneProjection> best;
  for (const Lane& lane : lanes) {
    if (lane.closed) {
      continue;
    }
    const std::optional<LaneProjection> candidate = ProjectOntoLane(lane, position);
    if (!candidate.has_value() || candidate->distance > config.max_lane_match_distance_m) {
      continue;
    }
    if (required_yaw.has_value() &&
        std::abs(NormalizeAngle(*required_yaw - candidate->yaw)) >
            config.max_lane_heading_difference_rad) {
      continue;
    }
    if (!best.has_value() || candidate->distance < best->distance) {
      best = candidate;
    }
  }
  return best;
}

void AppendPoint(std::vector<Vec2>* reference_line, const Vec2& point) {
  if (reference_line->empty() || Distance(reference_line->back(), point) > kAppendEpsilon) {
    reference_line->push_back(point);
  }
}

void AppendLaneSlice(std::vector<Vec2>* reference_line, const LaneProjection& start,
                     const LaneProjection& end) {
  AppendPoint(reference_line, start.position);
  for (size_t index = start.segment_index + 1; index <= end.segment_index; ++index) {
    AppendPoint(reference_line, start.lane->centerline[index]);
  }
  AppendPoint(reference_line, end.position);
}

}  // namespace

bool GlobalPlanner::Plan(const PlanningFrame& frame, GlobalRoute* route, std::string* error) const {
  if (route == nullptr || error == nullptr || frame.map == nullptr) {
    return false;
  }
  const std::vector<Lane>& lanes = frame.map->lanes;
  const ParkingSpot* spot = nullptr;
  for (const ParkingSpot& candidate : frame.map->parking_spots) {
    if (candidate.id == frame.target_parking_spot_id) {
      spot = &candidate;
      break;
    }
  }
  if (spot == nullptr) {
    *error = "target parking spot not found";
    return false;
  }

  const std::optional<LaneProjection> start =
      ClosestProjection(lanes, frame.ego.pose.position, frame.config, frame.ego.pose.yaw);
  if (!start.has_value()) {
    *error = "no open lane matches ego position and heading";
    return false;
  }
  const std::optional<LaneProjection> goal =
      ClosestProjection(lanes, spot->entry_pose.position, frame.config, std::nullopt);
  if (!goal.has_value()) {
    *error = "no open lane near parking entry";
    return false;
  }

  std::unordered_map<std::string, const Lane*> lane_by_id;
  for (const Lane& lane : lanes) {
    lane_by_id[lane.id] = &lane;
  }

  route->lane_ids.clear();
  if (start->lane->id == goal->lane->id) {
    if (goal->s + kGeometryEpsilon < start->s) {
      *error = "parking entry is behind ego on the same directed lane";
      return false;
    }
    route->lane_ids.push_back(start->lane->id);
  } else {
    const auto heuristic = [&lane_by_id, &goal](const std::string& lane_id) {
      if (lane_id == goal->lane->id) {
        return 0.0;
      }
      return Distance(lane_by_id.at(lane_id)->centerline.back(), goal->position);
    };
    struct QueueEntry {
      double priority = 0.0;
      double g_cost = 0.0;
      std::string id;
      bool operator>(const QueueEntry& other) const { return priority > other.priority; }
    };
    const double start_cost = LaneLength(*start->lane) - start->s;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    std::unordered_map<std::string, double> cost{{start->lane->id, start_cost}};
    std::unordered_map<std::string, std::string> parent;
    open.push({start_cost + heuristic(start->lane->id), start_cost, start->lane->id});

    while (!open.empty()) {
      const QueueEntry current = open.top();
      open.pop();
      if (current.g_cost != cost[current.id]) {
        continue;
      }
      if (current.id == goal->lane->id) {
        break;
      }
      const Lane* lane = lane_by_id.at(current.id);
      for (const std::string& successor : lane->successor_ids) {
        const Lane* successor_lane = lane_by_id.at(successor);
        if (successor_lane->closed) {
          continue;
        }
        const double traversed_length = successor == goal->lane->id
                                            ? goal->s
                                            : LaneLength(*successor_lane);
        const double next_cost = current.g_cost + traversed_length;
        if (!cost.contains(successor) || next_cost < cost[successor]) {
          cost[successor] = next_cost;
          parent[successor] = current.id;
          open.push({next_cost + heuristic(successor), next_cost, successor});
        }
      }
    }
    if (!cost.contains(goal->lane->id)) {
      *error = "parking entry is unreachable in lane graph";
      return false;
    }
    std::vector<std::string> reversed;
    for (std::string lane_id = goal->lane->id;; lane_id = parent.at(lane_id)) {
      reversed.push_back(lane_id);
      if (lane_id == start->lane->id) {
        break;
      }
    }
    route->lane_ids.assign(reversed.rbegin(), reversed.rend());
  }

  route->reference_line.clear();
  for (size_t index = 0; index < route->lane_ids.size(); ++index) {
    const Lane* lane = lane_by_id.at(route->lane_ids[index]);
    if (index == 0 && index + 1 == route->lane_ids.size()) {
      AppendLaneSlice(&route->reference_line, *start, *goal);
    } else if (index == 0) {
      AppendLaneSlice(&route->reference_line, *start, LaneEnd(*lane));
    } else if (index + 1 == route->lane_ids.size()) {
      AppendLaneSlice(&route->reference_line, LaneStart(*lane), *goal);
    } else {
      AppendLaneSlice(&route->reference_line, LaneStart(*lane), LaneEnd(*lane));
    }
  }
  AppendPoint(&route->reference_line, spot->entry_pose.position);

  std::vector<Pose2d> connection;
  if (!hybrid_a_star_.Plan(frame, spot->entry_pose, spot->target_pose, &connection, error)) {
    return false;
  }
  for (const Pose2d& point : connection) {
    AppendPoint(&route->reference_line, point.position);
  }
  if (route->reference_line.size() < 2) {
    *error = "reference line has no usable length";
    return false;
  }
  route->parking_target = spot->target_pose;
  return true;
}
}  // namespace avp
