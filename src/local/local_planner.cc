#include "local/local_planner.h"

#include <cmath>
#include <limits>

namespace avp {
namespace {

struct ReferencePoint {
  Vec2 position;
  double yaw = 0.0;
  double s = 0.0;
};

double Curvature(const Vec2& a, const Vec2& b, const Vec2& c) {
  const double ab = Distance(a, b);
  const double bc = Distance(b, c);
  const double ac = Distance(a, c);
  if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6) {
    return 0.0;
  }
  const double area2 = std::abs(((b.x - a.x) * (c.y - a.y)) -
                                ((b.y - a.y) * (c.x - a.x)));
  return 2.0 * area2 / (ab * bc * ac);
}

PredictionPoint PredictAt(const Obstacle& obstacle, uint64_t timestamp_ns) {
  PredictionPoint result = obstacle.prediction.front();
  for (const PredictionPoint& point : obstacle.prediction) {
    if (point.timestamp_ns > timestamp_ns) {
      break;
    }
    result = point;
  }
  return result;
}

std::vector<ReferencePoint> ResampleReferenceLine(const std::vector<Vec2>& reference_line,
                                                  double step_m) {
  std::vector<Vec2> positions{reference_line.front()};
  for (size_t i = 1; i < reference_line.size(); ++i) {
    const Vec2& start = reference_line[i - 1];
    const Vec2& end = reference_line[i];
    const double length = Distance(start, end);
    if (length < 1e-6) {
      continue;
    }
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    for (int sample = 1; sample <= count; ++sample) {
      positions.push_back(Interpolate(start, end, static_cast<double>(sample) / count));
    }
  }

  std::vector<ReferencePoint> result;
  result.reserve(positions.size());
  double s = 0.0;
  for (size_t i = 0; i < positions.size(); ++i) {
    if (i > 0) {
      s += Distance(positions[i - 1], positions[i]);
    }
    const Vec2& before = positions[i == 0 ? 0 : i - 1];
    const Vec2& after = positions[i + 1 < positions.size() ? i + 1 : i];
    result.push_back({positions[i], std::atan2(after.y - before.y, after.x - before.x), s});
  }
  return result;
}

Vec2 OffsetPosition(const ReferencePoint& reference, double lateral_offset_m) {
  return {reference.position.x - std::sin(reference.yaw) * lateral_offset_m,
          reference.position.y + std::cos(reference.yaw) * lateral_offset_m};
}

bool IsCollisionFree(const PlanningFrame& frame, const Pose2d& pose, double arrival_time_s) {
  const uint64_t timestamp = frame.header.timestamp_ns +
                             static_cast<uint64_t>(arrival_time_s * 1e9);
  for (const Obstacle& obstacle : frame.obstacles) {
    if (IsVehicleObstacleCollision(pose, frame.vehicle, PredictAt(obstacle, timestamp).pose,
                                   obstacle.length_m, obstacle.width_m)) {
      return false;
    }
  }
  return true;
}

double ArrivalTimeAt(const std::vector<double>& arrival_times, size_t index) {
  return arrival_times.empty() ? 0.0 : arrival_times[std::min(index, arrival_times.size() - 1)];
}

size_t StateIndex(int previous_lateral_index, int current_lateral_index, int lateral_count) {
  return static_cast<size_t>(previous_lateral_index * lateral_count + current_lateral_index);
}

}  // 匿名命名空间

bool LocalPlanner::Plan(const PlanningFrame& frame, const GlobalRoute& route,
                        const std::vector<double>& arrival_times, std::vector<PathPoint>* path,
                        std::string* error) const {
  if (path == nullptr || error == nullptr || route.reference_line.size() < 2) {
    return false;
  }

  const std::vector<ReferencePoint> reference =
      ResampleReferenceLine(route.reference_line, frame.config.path_step_m);
  if (reference.size() < 2) {
    *error = "reference line has no usable length";
    return false;
  }

  constexpr int kLateralCount = 13;
  constexpr double kLateralStepM = 0.3;
  constexpr double kLateralWeight = 1.0;
  constexpr double kSlopeWeight = 4.0;
  constexpr double kCurvatureWeight = 12.0;
  std::vector<double> lateral_offsets;
  lateral_offsets.reserve(kLateralCount);
  for (int index = 0; index < kLateralCount; ++index) {
    lateral_offsets.push_back((index - kLateralCount / 2) * kLateralStepM);
  }

  std::vector<std::vector<Vec2>> positions(reference.size(),
                                           std::vector<Vec2>(kLateralCount));
  for (size_t layer = 0; layer < reference.size(); ++layer) {
    for (int lateral = 0; lateral < kLateralCount; ++lateral) {
      positions[layer][lateral] = OffsetPosition(reference[layer], lateral_offsets[lateral]);
    }
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> initial_cost(kLateralCount, infinity);
  for (int lateral = 0; lateral < kLateralCount; ++lateral) {
    const Pose2d pose{positions[0][lateral], reference[0].yaw};
    if (IsCollisionFree(frame, pose, ArrivalTimeAt(arrival_times, 0))) {
      initial_cost[lateral] = kLateralWeight * lateral_offsets[lateral] * lateral_offsets[lateral];
    }
  }

  const double first_distance = Distance(positions[0][0], positions[1][0]);
  if (first_distance < 1e-6) {
    *error = "reference line contains duplicate samples";
    return false;
  }
  std::vector<double> state_cost(kLateralCount * kLateralCount, infinity);
  for (int first = 0; first < kLateralCount; ++first) {
    if (!std::isfinite(initial_cost[first])) {
      continue;
    }
    for (int second = 0; second < kLateralCount; ++second) {
      const Vec2 delta{positions[1][second].x - positions[0][first].x,
                       positions[1][second].y - positions[0][first].y};
      const Pose2d pose{positions[1][second], std::atan2(delta.y, delta.x)};
      if (!IsCollisionFree(frame, pose, ArrivalTimeAt(arrival_times, 1))) {
        continue;
      }
      const double slope = (lateral_offsets[second] - lateral_offsets[first]) / first_distance;
      state_cost[StateIndex(first, second, kLateralCount)] =
          initial_cost[first] + kLateralWeight * lateral_offsets[second] * lateral_offsets[second] +
          kSlopeWeight * slope * slope;
    }
  }

  std::vector<std::vector<int>> parents(reference.size(),
                                         std::vector<int>(kLateralCount * kLateralCount, -1));
  for (size_t layer = 2; layer < reference.size(); ++layer) {
    std::vector<double> next_cost(kLateralCount * kLateralCount, infinity);
    const double distance = Distance(positions[layer - 1][0], positions[layer][0]);
    if (distance < 1e-6) {
      *error = "reference line contains duplicate samples";
      return false;
    }
    for (int previous = 0; previous < kLateralCount; ++previous) {
      for (int current = 0; current < kLateralCount; ++current) {
        const double prior_cost = state_cost[StateIndex(previous, current, kLateralCount)];
        if (!std::isfinite(prior_cost)) {
          continue;
        }
        for (int next = 0; next < kLateralCount; ++next) {
          const Vec2 delta{positions[layer][next].x - positions[layer - 1][current].x,
                           positions[layer][next].y - positions[layer - 1][current].y};
          const Pose2d pose{positions[layer][next], std::atan2(delta.y, delta.x)};
          if (!IsCollisionFree(frame, pose, ArrivalTimeAt(arrival_times, layer))) {
            continue;
          }
          const double curvature = Curvature(positions[layer - 2][previous],
                                             positions[layer - 1][current], positions[layer][next]);
          if (curvature > frame.vehicle.max_curvature_1pm) {
            continue;
          }
          const double slope = (lateral_offsets[next] - lateral_offsets[current]) / distance;
          const double candidate = prior_cost +
                                   kLateralWeight * lateral_offsets[next] * lateral_offsets[next] +
                                   kSlopeWeight * slope * slope +
                                   kCurvatureWeight * curvature * curvature;
          const size_t next_state = StateIndex(current, next, kLateralCount);
          if (candidate < next_cost[next_state]) {
            next_cost[next_state] = candidate;
            parents[layer][next_state] = previous;
          }
        }
      }
    }
    state_cost = std::move(next_cost);
  }

  int previous = -1;
  int current = -1;
  double best_cost = infinity;
  for (int first = 0; first < kLateralCount; ++first) {
    for (int second = 0; second < kLateralCount; ++second) {
      const double cost = state_cost[StateIndex(first, second, kLateralCount)];
      if (cost < best_cost) {
        best_cost = cost;
        previous = first;
        current = second;
      }
    }
  }
  if (!std::isfinite(best_cost)) {
    *error = "no feasible S-L lattice path";
    return false;
  }

  std::vector<int> lateral_indices(reference.size());
  lateral_indices.back() = current;
  lateral_indices[reference.size() - 2] = previous;
  for (size_t layer = reference.size() - 1; layer >= 2; --layer) {
    const int before = parents[layer][StateIndex(previous, current, kLateralCount)];
    if (before < 0) {
      *error = "S-L lattice backtracking failed";
      return false;
    }
    lateral_indices[layer - 2] = before;
    current = previous;
    previous = before;
  }

  path->clear();
  path->reserve(reference.size());
  double s = 0.0;
  for (size_t index = 0; index < reference.size(); ++index) {
    const Vec2 position = positions[index][lateral_indices[index]];
    if (index > 0) {
      s += Distance(path->back().position, position);
    }
    const Vec2& before = index == 0 ? position : path->back().position;
    const Vec2& after = index + 1 < reference.size()
                            ? positions[index + 1][lateral_indices[index + 1]]
                            : position;
    const double yaw = index + 1 < reference.size() ? std::atan2(after.y - before.y, after.x - before.x)
                                                      : path->back().yaw;
    path->push_back({position, yaw, 0.0, s});
  }
  for (size_t index = 1; index + 1 < path->size(); ++index) {
    path->at(index).curvature = Curvature(path->at(index - 1).position, path->at(index).position,
                                          path->at(index + 1).position);
  }
  for (size_t index = 0; index < path->size(); ++index) {
    if (path->at(index).curvature > frame.vehicle.max_curvature_1pm ||
        !IsCollisionFree(frame, {path->at(index).position, path->at(index).yaw},
                         ArrivalTimeAt(arrival_times, index))) {
      *error = "selected S-L lattice path is invalid";
      path->clear();
      return false;
    }
  }
  return true;
}
}  // avp 命名空间
