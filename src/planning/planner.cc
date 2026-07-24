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
  // 始终回传请求头，便于上层将成功或失败结果与原始请求关联。
  PlanningResponse response;
  response.header = request.header;

  // 在进入各规划算法前，统一校验外部输入并归一化为内部 PlanningFrame。
  PlanningFrame frame;
  std::string error;
  if (!adapter_.Adapt(request, vehicle_, config_, &frame, &error)) {
    response.status = PlanningStatus::kInvalidInput;
    response.message = error;
    return response;
  }

  // 全局规划先在车道图中找到通往车位入口的路线，并用 Hybrid A* 补齐到目标车位的连接。
  // 这一步失败意味着没有可供局部规划器跟随的参考线。
  GlobalRoute route;
  if (!global_planner_.Plan(frame, &route, &error)) {
    response.status = PlanningStatus::kNoRoute;
    response.message = error;
    return response;
  }

  // 路径与速度相互依赖：路径避障需要预计到达时刻，速度规划又以路径弧长为输入。
  // 首轮没有到达时刻，局部规划器按 t=0 检查；之后将速度剖面回投为每个路径点的到达时刻，
  // 供下一轮在对应的障碍物预测时刻重新规划路径。
  std::vector<double> arrivals;
  std::vector<PathPoint> path;
  std::vector<SpeedPoint> speed;
  for (int iteration = 0; iteration < frame.config.path_coupling_iterations; ++iteration) {
    if (!local_planner_.Plan(frame, route, arrivals, &path, &error) ||
        !speed_planner_.Plan(frame, path, &speed, &error))
      break;

    // 对每个路径弧长 s，取速度剖面中第一个到达或越过该位置的时刻。
    // 若该轮速度剖面未覆盖路径末端，则保守地使用整个规划时域末尾。
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

  // 任一局部模块未产生可行结果时，不能输出半成品轨迹。生成沿当前车头方向的匀减速停车
  // 轨迹作为降级，并显式记录它是否也通过碰撞检查，供上层决定后续安全动作。
  if (path.empty() || speed.empty()) {
    response.trajectory = MakeStopTrajectory(frame);
    const bool stop_is_safe = IsCollisionFree(frame, response.trajectory);
    response.status = PlanningStatus::kNoSafeTrajectory;
    response.message = error.empty() ? "no feasible local path or speed profile" : error;
    response.diagnostics.push_back("fallback=emergency_stop");
    response.diagnostics.push_back(stop_is_safe ? "stop_collision_free=true"
                                                : "stop_collision_free=false");
    return response;
  }

  // 速度剖面只描述“何时到达哪个弧长”；在这里从空间路径插值得到位姿和曲率，形成
  // 控制器可直接消费的时序轨迹。
  response.trajectory.reserve(speed.size());
  for (const SpeedPoint& point : speed) {
    response.trajectory.push_back({{PositionAtS(path, point.s), YawAtS(path, point.s)},
                                   CurvatureAtS(path, point.s),
                                   point.speed_mps,
                                   point.acceleration_mps2,
                                   point.time_s});
  }

  // 局部路径和速度规划各自做过离散碰撞检查，但合成后仍需以最终轨迹逐点校验，作为
  // 输出前的统一安全闸门。失败时同样退化为停车轨迹，而不返回有碰撞风险的规划结果。
  if (!IsCollisionFree(frame, response.trajectory)) {
    response.trajectory = MakeStopTrajectory(frame);
    response.status = PlanningStatus::kNoSafeTrajectory;
    response.message = "post-plan collision validation failed";
    response.diagnostics.push_back("fallback=emergency_stop");
    response.diagnostics.push_back(IsCollisionFree(frame, response.trajectory)
                                       ? "stop_collision_free=true"
                                       : "stop_collision_free=false");
    return response;
  }

  // 只有完整时序轨迹通过最终校验，才报告规划成功及所用算法链路。
  response.status = PlanningStatus::kOk;
  response.message = "spatiotemporal DP trajectory generated";
  response.diagnostics.push_back("global=A_STAR");
  response.diagnostics.push_back("local=SL_DP");
  response.diagnostics.push_back("speed=ST_DP");
  return response;
}
}  // namespace avp
