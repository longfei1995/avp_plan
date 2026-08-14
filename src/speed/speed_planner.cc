#include "speed/speed_planner.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace avp {
namespace {
constexpr double kConstraintTolerance = 1e-9;
constexpr double kTerminalPositionToleranceM = 0.02;
constexpr double kJerkSampleStepMps3 = 1.0;
constexpr double kSKeyResolutionM = 0.01;
constexpr double kSpeedKeyResolutionMps = 0.02;
constexpr double kAccelerationKeyResolutionMps2 = 0.1;
constexpr double kSpeedWeight = 4.0;
constexpr double kAccelerationWeight = 0.2;
constexpr size_t kMaxStatesPerLayer = 400;

using StateKey = std::array<int64_t, 3>;

struct LongitudinalState {
  double s = 0.0;
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
};

struct Node {
  LongitudinalState state;
  double cost = std::numeric_limits<double>::infinity();
  double jerk_mps3 = 0.0;
  StateKey parent{};
};

using Layer = std::map<StateKey, Node>;

PredictionPoint PredictAt(const Obstacle& obstacle, uint64_t timestamp_ns) {
  PredictionPoint result = obstacle.prediction.front();
  for (const PredictionPoint& point : obstacle.prediction) {
    if (point.timestamp_ns > timestamp_ns) break;
    result = point;
  }
  return result;
}

Pose2d PoseAtS(const std::vector<PathPoint>& path, double s) {
  if (s <= path.front().s) return {path.front().position, path.front().yaw};
  for (size_t index = 1; index < path.size(); ++index) {
    if (s > path[index].s) continue;
    const double span = path[index].s - path[index - 1].s;
    const double ratio = span <= 1e-9 ? 0.0 : (s - path[index - 1].s) / span;
    return {Interpolate(path[index - 1].position, path[index].position, ratio),
            NormalizeAngle(path[index - 1].yaw +
                           ratio * NormalizeAngle(path[index].yaw - path[index - 1].yaw))};
  }
  return {path.back().position, path.back().yaw};
}

bool IsBlocked(const PlanningFrame& frame, const std::vector<PathPoint>& path, double s,
               uint64_t timestamp_ns) {
  const Pose2d pose = PoseAtS(path, s);
  for (const Obstacle& obstacle : frame.obstacles) {
    if (IsVehicleObstacleCollision(pose, frame.vehicle, PredictAt(obstacle, timestamp_ns).pose,
                                   obstacle.length_m, obstacle.width_m)) {
      return true;
    }
  }
  return false;
}

int64_t Quantize(double value, double resolution) {
  return static_cast<int64_t>(std::llround(value / resolution));
}

StateKey MakeKey(const LongitudinalState& state) {
  return {Quantize(state.s, kSKeyResolutionM),
          Quantize(state.speed_mps, kSpeedKeyResolutionMps),
          Quantize(state.acceleration_mps2, kAccelerationKeyResolutionMps2)};
}

bool IsTransitionFeasible(const PlanningFrame& frame, const LongitudinalState& next,
                          double jerk_mps3, double max_speed_mps, double path_length_m) {
  return next.s >= -kConstraintTolerance && next.s <= path_length_m + kConstraintTolerance &&
         next.speed_mps >= -kConstraintTolerance &&
         next.speed_mps <= max_speed_mps + kConstraintTolerance &&
         next.acceleration_mps2 <=
             frame.vehicle.max_acceleration_mps2 + kConstraintTolerance &&
         next.acceleration_mps2 >=
             -frame.vehicle.max_deceleration_mps2 - kConstraintTolerance &&
         jerk_mps3 <= frame.vehicle.max_jerk_mps3 + kConstraintTolerance &&
         jerk_mps3 >= frame.vehicle.min_jerk_mps3 - kConstraintTolerance;
}

bool HasStoppingRoom(const PlanningFrame& frame, const LongitudinalState& state,
                     double path_length_m) {
  const double remaining_s = std::max(0.0, path_length_m - state.s);
  if (remaining_s <= kTerminalPositionToleranceM) {
    const double time_step = frame.config.time_step_s;
    const double stop_acceleration = -state.speed_mps / time_step;
    const double stop_jerk = (stop_acceleration - state.acceleration_mps2) / time_step;
    const double release_jerk = -stop_acceleration / time_step;
    return stop_jerk >= frame.vehicle.min_jerk_mps3 - kConstraintTolerance &&
           stop_jerk <= frame.vehicle.max_jerk_mps3 + kConstraintTolerance &&
           release_jerk <= frame.vehicle.max_jerk_mps3 + kConstraintTolerance;
  }
  const double braking_distance_lower_bound =
      state.speed_mps * state.speed_mps / (2.0 * frame.vehicle.max_deceleration_mps2);
  return braking_distance_lower_bound <= remaining_s + kTerminalPositionToleranceM;
}

double ReferenceSpeed(const PlanningFrame& frame, double max_speed_mps, double remaining_s) {
  return std::min(max_speed_mps,
                  std::sqrt(std::max(
                      0.0, 2.0 * frame.vehicle.max_deceleration_mps2 * remaining_s)));
}

double TransitionCost(const PlanningFrame& frame, const LongitudinalState& next,
                      double jerk_mps3, double reference_speed_mps, double max_speed_mps) {
  const double speed_scale = std::max(0.1, max_speed_mps);
  const double acceleration_scale = next.acceleration_mps2 >= 0.0
                                        ? frame.vehicle.max_acceleration_mps2
                                        : frame.vehicle.max_deceleration_mps2;
  const double jerk_scale =
      jerk_mps3 >= 0.0 ? frame.vehicle.max_jerk_mps3 : -frame.vehicle.min_jerk_mps3;
  const double normalized_speed_error =
      (next.speed_mps - reference_speed_mps) / speed_scale;
  const double normalized_acceleration = next.acceleration_mps2 / acceleration_scale;
  const double normalized_jerk = jerk_mps3 / jerk_scale;
  return frame.config.time_step_s *
         (kSpeedWeight * normalized_speed_error * normalized_speed_error +
          kAccelerationWeight * normalized_acceleration * normalized_acceleration +
          frame.config.jerk_weight * normalized_jerk * normalized_jerk);
}

void AddCandidate(double jerk_mps3, const PlanningFrame& frame, std::set<double>* candidates) {
  if (jerk_mps3 >= frame.vehicle.min_jerk_mps3 - kConstraintTolerance &&
      jerk_mps3 <= frame.vehicle.max_jerk_mps3 + kConstraintTolerance) {
    candidates->insert(std::clamp(jerk_mps3, frame.vehicle.min_jerk_mps3,
                                  frame.vehicle.max_jerk_mps3));
  }
}

std::set<double> CandidateJerks(const PlanningFrame& frame, const LongitudinalState& state,
                                double max_speed_mps, double path_length_m) {
  std::set<double> result;
  const double time_step = frame.config.time_step_s;
  const int first_sample =
      static_cast<int>(std::ceil(frame.vehicle.min_jerk_mps3 / kJerkSampleStepMps3));
  const int last_sample =
      static_cast<int>(std::floor(frame.vehicle.max_jerk_mps3 / kJerkSampleStepMps3));
  for (int sample = first_sample; sample <= last_sample; ++sample) {
    AddCandidate(sample * kJerkSampleStepMps3, frame, &result);
  }
  AddCandidate(frame.vehicle.min_jerk_mps3, frame, &result);
  AddCandidate(frame.vehicle.max_jerk_mps3, frame, &result);
  AddCandidate(0.0, frame, &result);
  AddCandidate(-state.acceleration_mps2 / time_step, frame, &result);
  AddCandidate((frame.vehicle.max_acceleration_mps2 - state.acceleration_mps2) / time_step,
               frame, &result);
  AddCandidate((-frame.vehicle.max_deceleration_mps2 - state.acceleration_mps2) / time_step,
               frame, &result);

  const double remaining_s = std::max(0.0, path_length_m - state.s);
  const double reference_speed = ReferenceSpeed(frame, max_speed_mps, remaining_s);
  const double reference_acceleration = (reference_speed - state.speed_mps) / time_step;
  AddCandidate((reference_acceleration - state.acceleration_mps2) / time_step, frame, &result);

  const double endpoint_speed = remaining_s / time_step;
  const double endpoint_acceleration = (endpoint_speed - state.speed_mps) / time_step;
  AddCandidate((endpoint_acceleration - state.acceleration_mps2) / time_step, frame, &result);

  const double stop_acceleration = -state.speed_mps / time_step;
  AddCandidate((stop_acceleration - state.acceleration_mps2) / time_step, frame, &result);
  return result;
}

bool PreferCandidate(const Node& candidate, const Node& existing) {
  constexpr double kCostTolerance = 1e-12;
  if (candidate.cost < existing.cost - kCostTolerance) return true;
  if (candidate.cost > existing.cost + kCostTolerance) return false;
  if (std::abs(candidate.jerk_mps3) < std::abs(existing.jerk_mps3) - kCostTolerance) return true;
  if (std::abs(candidate.jerk_mps3) > std::abs(existing.jerk_mps3) + kCostTolerance) return false;
  if (candidate.state.s > existing.state.s + kConstraintTolerance) return true;
  if (candidate.state.s < existing.state.s - kConstraintTolerance) return false;
  return candidate.parent < existing.parent;
}

bool IsStoppedAtEnd(const LongitudinalState& state, double path_length_m) {
  return path_length_m - state.s <= kTerminalPositionToleranceM &&
         std::abs(state.speed_mps) <= kConstraintTolerance &&
         std::abs(state.acceleration_mps2) <= kConstraintTolerance;
}

void PruneLayer(Layer* layer) {
  if (layer->size() <= kMaxStatesPerLayer) return;
  std::vector<std::pair<StateKey, Node>> by_cost(layer->begin(), layer->end());
  std::sort(by_cost.begin(), by_cost.end(), [](const auto& left, const auto& right) {
    constexpr double kCostTolerance = 1e-12;
    if (left.second.cost < right.second.cost - kCostTolerance) return true;
    if (left.second.cost > right.second.cost + kCostTolerance) return false;
    if (std::abs(left.second.jerk_mps3) <
        std::abs(right.second.jerk_mps3) - kCostTolerance) {
      return true;
    }
    if (std::abs(left.second.jerk_mps3) >
        std::abs(right.second.jerk_mps3) + kCostTolerance) {
      return false;
    }
    if (left.second.state.s > right.second.state.s + kConstraintTolerance) return true;
    if (left.second.state.s < right.second.state.s - kConstraintTolerance) return false;
    return left.first < right.first;
  });
  std::vector<std::pair<StateKey, Node>> by_speed = by_cost;
  std::sort(by_speed.begin(), by_speed.end(), [](const auto& left, const auto& right) {
    if (left.second.state.speed_mps < right.second.state.speed_mps - kConstraintTolerance) {
      return true;
    }
    if (left.second.state.speed_mps > right.second.state.speed_mps + kConstraintTolerance) {
      return false;
    }
    if (left.second.state.s > right.second.state.s + kConstraintTolerance) return true;
    if (left.second.state.s < right.second.state.s - kConstraintTolerance) return false;
    return left.first < right.first;
  });
  std::vector<std::pair<StateKey, Node>> by_progress = by_cost;
  std::sort(by_progress.begin(), by_progress.end(), [](const auto& left, const auto& right) {
    if (left.second.state.s > right.second.state.s + kConstraintTolerance) return true;
    if (left.second.state.s < right.second.state.s - kConstraintTolerance) return false;
    if (left.second.state.speed_mps < right.second.state.speed_mps - kConstraintTolerance) {
      return true;
    }
    if (left.second.state.speed_mps > right.second.state.speed_mps + kConstraintTolerance) {
      return false;
    }
    return left.first < right.first;
  });

  std::set<StateKey> selected;
  const auto select = [&selected](const auto& ordered, size_t count) {
    for (size_t index = 0; index < std::min(count, ordered.size()); ++index) {
      selected.insert(ordered[index].first);
    }
  };
  select(by_cost, 240);
  select(by_speed, 80);
  select(by_progress, 80);
  for (const auto& [key, node] : by_cost) {
    static_cast<void>(node);
    if (selected.size() >= kMaxStatesPerLayer) break;
    selected.insert(key);
  }
  Layer pruned;
  for (const StateKey& key : selected) {
    pruned.emplace(key, layer->at(key));
  }
  *layer = std::move(pruned);
}
}  // namespace

bool SpeedPlanner::Plan(const PlanningFrame& frame, const std::vector<PathPoint>& path,
                        std::vector<SpeedPoint>* profile, std::string* error,
                        const SpeedPlanOptions& options) const {
  if (profile == nullptr || error == nullptr || path.empty()) return false;
  profile->clear();
  const int time_count =
      static_cast<int>(std::floor(frame.config.horizon_s / frame.config.time_step_s)) + 1;
  if (time_count < 2) {
    *error = "speed horizon must contain at least two time steps";
    return false;
  }
  const double max_speed = std::min(frame.vehicle.max_speed_mps, options.max_speed_mps);
  if (!std::isfinite(max_speed) || max_speed <= 0.0) {
    *error = "speed limit must be positive and finite";
    return false;
  }
  const double path_length = path.back().s;
  if (path_length < -kConstraintTolerance) {
    *error = "speed path length must be non-negative";
    return false;
  }
  if (IsBlocked(frame, path, 0.0, frame.header.timestamp_ns)) {
    *error = "initial S-T lattice state is blocked";
    return false;
  }

  const LongitudinalState initial{0.0, std::abs(frame.ego.speed_mps),
                                  frame.ego.acceleration_mps2};
  std::vector<Layer> layers(static_cast<size_t>(time_count));
  const StateKey initial_key = MakeKey(initial);
  layers.front().emplace(initial_key, Node{initial, 0.0, 0.0, initial_key});

  const double time_step = frame.config.time_step_s;
  for (int time_index = 1; time_index < time_count; ++time_index) {
    Layer& next_layer = layers[static_cast<size_t>(time_index)];
    const uint64_t timestamp =
        frame.header.timestamp_ns + static_cast<uint64_t>(time_index * time_step * 1e9);
    for (const auto& [parent_key, parent] : layers[static_cast<size_t>(time_index - 1)]) {
      for (const double jerk : CandidateJerks(frame, parent.state, max_speed, path_length)) {
        LongitudinalState next;
        next.acceleration_mps2 = parent.state.acceleration_mps2 + jerk * time_step;
        next.speed_mps = parent.state.speed_mps + next.acceleration_mps2 * time_step;
        next.s = parent.state.s + next.speed_mps * time_step;
        if (!IsTransitionFeasible(frame, next, jerk, max_speed, path_length)) continue;
        next.speed_mps = std::max(0.0, next.speed_mps);
        next.s = std::clamp(next.s, 0.0, path_length);
        if ((options.require_stop_at_end || time_index + 1 < time_count) &&
            !HasStoppingRoom(frame, next, path_length)) {
          continue;
        }
        if (IsBlocked(frame, path, next.s, timestamp)) continue;

        const double remaining_s = std::max(0.0, path_length - next.s);
        const double reference_speed = ReferenceSpeed(frame, max_speed, remaining_s);
        Node candidate{next,
                       parent.cost +
                           TransitionCost(frame, next, jerk, reference_speed, max_speed),
                       jerk, parent_key};
        const StateKey key = MakeKey(next);
        const auto existing = next_layer.find(key);
        if (existing == next_layer.end() || PreferCandidate(candidate, existing->second)) {
          next_layer[key] = candidate;
        }
      }
    }
    if (next_layer.empty()) {
      *error = "all acceleration- and jerk-feasible S-T lattice states are blocked";
      return false;
    }
    PruneLayer(&next_layer);
  }

  const Layer& final_layer = layers.back();
  auto best = final_layer.end();
  for (auto candidate = final_layer.begin(); candidate != final_layer.end(); ++candidate) {
    if (options.require_stop_at_end && !IsStoppedAtEnd(candidate->second.state, path_length)) {
      continue;
    }
    if (best == final_layer.end() || PreferCandidate(candidate->second, best->second)) {
      best = candidate;
    }
  }
  if (best == final_layer.end()) {
    *error = "path end is not reachable with zero speed and acceleration";
    return false;
  }

  std::vector<LongitudinalState> states(static_cast<size_t>(time_count));
  StateKey key = best->first;
  for (int time_index = time_count - 1; time_index >= 0; --time_index) {
    const Node& node = layers[static_cast<size_t>(time_index)].at(key);
    states[static_cast<size_t>(time_index)] = node.state;
    key = node.parent;
  }

  profile->reserve(static_cast<size_t>(time_count));
  for (int time_index = 0; time_index < time_count; ++time_index) {
    const LongitudinalState& state = states[static_cast<size_t>(time_index)];
    profile->push_back(
        {time_index * time_step, state.s, state.speed_mps, state.acceleration_mps2});
  }
  return true;
}
}  // namespace avp
