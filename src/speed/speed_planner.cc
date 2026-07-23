#include "speed/speed_planner.h"

#include <cmath>
#include <limits>

namespace avp {
namespace {

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

bool IsBlocked(const PlanningFrame& frame, const PathPoint& point, uint64_t timestamp_ns) {
  const Pose2d pose{point.position, point.yaw};
  for (const Obstacle& obstacle : frame.obstacles) {
    if (IsVehicleObstacleCollision(pose, frame.vehicle,
                                   PredictAt(obstacle, timestamp_ns).pose, obstacle.length_m,
                                   obstacle.width_m)) {
      return true;
    }
  }
  return false;
}

size_t StateIndex(int previous_path_index, int current_path_index, int path_count) {
  return static_cast<size_t>(previous_path_index * path_count + current_path_index);
}

double TransitionCost(double speed_mps, double acceleration_mps2, double desired_speed_mps,
                      double delta_s) {
  return 0.2 * std::pow(speed_mps - desired_speed_mps, 2.0) +
         0.02 * std::pow(delta_s, 2.0) + 0.05 * std::pow(acceleration_mps2, 2.0);
}

}  // 匿名命名空间

bool SpeedPlanner::Plan(const PlanningFrame& frame, const std::vector<PathPoint>& path,
                        std::vector<SpeedPoint>* profile, std::string* error) const {
  if (profile == nullptr || error == nullptr || path.empty()) {
    return false;
  }
  const int time_count =
      static_cast<int>(std::floor(frame.config.horizon_s / frame.config.time_step_s)) + 1;
  const int path_count = static_cast<int>(path.size());
  if (time_count < 2) {
    *error = "speed horizon must contain at least two time steps";
    return false;
  }
  if (IsBlocked(frame, path.front(), frame.header.timestamp_ns)) {
    *error = "initial S-T lattice state is blocked";
    return false;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  const double time_step = frame.config.time_step_s;
  const double desired_speed = std::min(frame.vehicle.max_speed_mps, 1.5);
  std::vector<double> state_cost(path_count * path_count, infinity);
  for (int current = 0; current < path_count; ++current) {
    const double delta_s = path[current].s - path.front().s;
    const double speed = delta_s / time_step;
    const double acceleration = (speed - frame.ego.speed_mps) / time_step;
    const uint64_t timestamp = frame.header.timestamp_ns + static_cast<uint64_t>(time_step * 1e9);
    if (speed > frame.vehicle.max_speed_mps + 1e-9 ||
        acceleration > frame.vehicle.max_acceleration_mps2 + 1e-9 ||
        acceleration < -frame.vehicle.max_deceleration_mps2 - 1e-9 ||
        IsBlocked(frame, path[current], timestamp)) {
      continue;
    }
    state_cost[StateIndex(0, current, path_count)] =
        TransitionCost(speed, acceleration, desired_speed, delta_s);
  }

  std::vector<std::vector<int>> parents(time_count,
                                        std::vector<int>(path_count * path_count, -1));
  for (int time_index = 2; time_index < time_count; ++time_index) {
    const uint64_t timestamp = frame.header.timestamp_ns +
                               static_cast<uint64_t>(time_index * time_step * 1e9);
    std::vector<double> next_cost(path_count * path_count, infinity);
    for (int previous = 0; previous < path_count; ++previous) {
      for (int current = previous; current < path_count; ++current) {
        const double prior_cost = state_cost[StateIndex(previous, current, path_count)];
        if (!std::isfinite(prior_cost)) {
          continue;
        }
        const double previous_speed = (path[current].s - path[previous].s) / time_step;
        for (int next = current; next < path_count; ++next) {
          const double delta_s = path[next].s - path[current].s;
          const double speed = delta_s / time_step;
          if (speed > frame.vehicle.max_speed_mps + 1e-9) {
            break;
          }
          const double acceleration = (speed - previous_speed) / time_step;
          if (acceleration > frame.vehicle.max_acceleration_mps2 + 1e-9 ||
              acceleration < -frame.vehicle.max_deceleration_mps2 - 1e-9 ||
              IsBlocked(frame, path[next], timestamp)) {
            continue;
          }
          const double candidate = prior_cost +
                                   TransitionCost(speed, acceleration, desired_speed, delta_s);
          const size_t next_state = StateIndex(current, next, path_count);
          if (candidate < next_cost[next_state]) {
            next_cost[next_state] = candidate;
            parents[time_index][next_state] = previous;
          }
        }
      }
    }
    state_cost = std::move(next_cost);
  }

  int previous = -1;
  int current = -1;
  double final_cost = infinity;
  for (int first = 0; first < path_count; ++first) {
    for (int second = first; second < path_count; ++second) {
      const double cost = state_cost[StateIndex(first, second, path_count)];
      const double progress_reward = 0.1 * path[second].s;
      if (cost - progress_reward < final_cost) {
        final_cost = cost - progress_reward;
        previous = first;
        current = second;
      }
    }
  }
  if (!std::isfinite(final_cost)) {
    *error = "all acceleration-feasible S-T lattice states are blocked";
    return false;
  }

  std::vector<int> indices(time_count);
  indices.back() = current;
  indices[time_count - 2] = previous;
  for (int time_index = time_count - 1; time_index >= 2; --time_index) {
    const int before = parents[time_index][StateIndex(previous, current, path_count)];
    if (before < 0) {
      *error = "S-T lattice backtracking failed";
      return false;
    }
    indices[time_index - 2] = before;
    current = previous;
    previous = before;
  }
  if (indices.front() != 0) {
    *error = "S-T lattice does not start at the path origin";
    return false;
  }

  profile->clear();
  profile->reserve(time_count);
  double previous_speed = frame.ego.speed_mps;
  for (int time_index = 0; time_index < time_count; ++time_index) {
    const double speed = time_index == 0
                             ? frame.ego.speed_mps
                             : (path[indices[time_index]].s - path[indices[time_index - 1]].s) /
                                   time_step;
    const double acceleration = time_index == 0
                                    ? frame.ego.acceleration_mps2
                                    : (speed - previous_speed) / time_step;
    profile->push_back({time_index * time_step, path[indices[time_index]].s, speed, acceleration});
    previous_speed = speed;
  }
  return true;
}
}  // avp 命名空间
