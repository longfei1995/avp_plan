#include "planning/planner.h"

#include <algorithm>

namespace avp {
namespace {
Vec2 PositionAtS(const std::vector<PathPoint>& path, double s) {
  if (s <= path.front().s) return path.front().position;
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) {
      const double span = path[i].s - path[i - 1].s;
      return Interpolate(path[i - 1].position, path[i].position,
                         span < 1e-6 ? 0.0 : (s - path[i - 1].s) / span);
    }
  }
  return path.back().position;
}
double YawAtS(const std::vector<PathPoint>& path, double s) {
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) return path[i - 1].yaw;
  }
  return path.back().yaw;
}
double CurvatureAtS(const std::vector<PathPoint>& path, double s) {
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) return path[i - 1].curvature;
  }
  return path.back().curvature;
}
PredictionPoint PredictAt(const Obstacle& obstacle, uint64_t timestamp_ns) {
  PredictionPoint result = obstacle.prediction.front();
  for (const PredictionPoint& point : obstacle.prediction) {
    if (point.timestamp_ns > timestamp_ns) break;
    result = point;
  }
  return result;
}
bool IsCollisionFree(const PlanningFrame& frame,
                     const std::vector<TimedTrajectoryPoint>& trajectory) {
  for (const TimedTrajectoryPoint& point : trajectory) {
    const uint64_t timestamp =
        frame.header.timestamp_ns + static_cast<uint64_t>(point.relative_time_s * 1e9);
    for (const Obstacle& obstacle : frame.obstacles) {
      if (IsVehicleObstacleCollision(point.pose, frame.vehicle, PredictAt(obstacle, timestamp).pose,
                                     obstacle.length_m, obstacle.width_m)) {
        return false;
      }
    }
  }
  return true;
}
std::vector<TimedTrajectoryPoint> MakeStopTrajectory(const PlanningFrame& frame) {
  std::vector<TimedTrajectoryPoint> result;
  const double deceleration = std::max(0.1, frame.vehicle.max_deceleration_mps2);
  const double stop_time =
      std::max(frame.config.time_step_s, std::abs(frame.ego.speed_mps) / deceleration);
  for (double t = 0.0; t <= stop_time + 1e-9; t += frame.config.time_step_s) {
    const double speed = std::max(0.0, frame.ego.speed_mps - deceleration * t);
    const double distance = frame.ego.speed_mps * t - 0.5 * deceleration * t * t;
    const Vec2 position{
        frame.ego.pose.position.x + std::cos(frame.ego.pose.yaw) * std::max(0.0, distance),
        frame.ego.pose.position.y + std::sin(frame.ego.pose.yaw) * std::max(0.0, distance)};
    result.push_back({{position, frame.ego.pose.yaw}, 0.0, speed, -deceleration, t});
  }
  return result;
}
}  // namespace

Planner::Planner(VehicleConfig vehicle, PlannerConfig config)
    : vehicle_(vehicle), config_(config) {}

PlanningResponse Planner::Plan(const PlanningRequest& request) const {
  // 回传请求头。
  PlanningResponse response;
  response.header = request.header;

  // 输入适配与校验。
  PlanningFrame frame;
  std::string error;
  if (!adapter_.Adapt(request, vehicle_, config_, &frame, &error)) {
    response.status = PlanningStatus::kInvalidInput;
    response.message = error;
    return response;
  }

  // 全局路径规划。
  GlobalRoute route;
  if (!global_planner_.Plan(frame, &route, &error)) {
    response.status = PlanningStatus::kNoRoute;
    response.message = error;
    return response;
  }

  // 路径-速度迭代耦合。
  std::vector<double> arrivals;
  std::vector<PathPoint> path;
  std::vector<SpeedPoint> speed;
  for (int iteration = 0; iteration < frame.config.path_coupling_iterations; ++iteration) {
    if (!local_planner_.Plan(frame, route, arrivals, &path, &error) ||
        !speed_planner_.Plan(frame, path, &speed, &error))
      break;

    // 回投路径点到达时刻。
    arrivals.assign(path.size(), frame.config.horizon_s);
    for (size_t i = 0; i < path.size(); ++i) {
      for (const SpeedPoint& point : speed) {
        if (point.s + 1e-6 >= path[i].s) {
          arrivals[i] = point.time_s;
          break;
        }
      }
    }
  }

  // 局部规划失败时降级停车。
  if (path.empty() || speed.empty()) {
    response.trajectory = MakeStopTrajectory(frame);
    const bool stop_is_safe = IsCollisionFree(frame, response.trajectory);
    response.status = PlanningStatus::kNoSafeTrajectory;
    response.message = error.empty() ? "no feasible local path or speed profile" : error;
    response.diagnostics.push_back("fallback=emergency_stop");
    response.diagnostics.push_back("jerk_constraint=emergency_exempt");
    response.diagnostics.push_back(stop_is_safe ? "stop_collision_free=true"
                                                : "stop_collision_free=false");
    return response;
  }

  // 合成时序轨迹。
  response.trajectory.reserve(speed.size());
  for (const SpeedPoint& point : speed) {
    response.trajectory.push_back({{PositionAtS(path, point.s), YawAtS(path, point.s)},
                                   CurvatureAtS(path, point.s),
                                   point.speed_mps,
                                   point.acceleration_mps2,
                                   point.time_s});
  }

  // 最终碰撞校验。
  if (!IsCollisionFree(frame, response.trajectory)) {
    response.trajectory = MakeStopTrajectory(frame);
    response.status = PlanningStatus::kNoSafeTrajectory;
    response.message = "post-plan collision validation failed";
    response.diagnostics.push_back("fallback=emergency_stop");
    response.diagnostics.push_back("jerk_constraint=emergency_exempt");
    response.diagnostics.push_back(IsCollisionFree(frame, response.trajectory)
                                       ? "stop_collision_free=true"
                                       : "stop_collision_free=false");
    return response;
  }

  // 返回规划结果。
  response.status = PlanningStatus::kOk;
  response.message = "spatiotemporal DP trajectory generated";
  response.diagnostics.push_back("global=A_STAR");
  response.diagnostics.push_back("local=SL_DP");
  response.diagnostics.push_back("speed=ST_DP");
  return response;
}
}  // namespace avp
