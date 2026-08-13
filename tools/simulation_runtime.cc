#include "tools/simulation_runtime.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace avp::tools {
namespace {
constexpr double kSimulationStepS = 0.02;
constexpr double kPlanningPeriodS = 0.2;

const TimedTrajectoryPoint* ReferenceAt(const PlanningResponse& response, double relative_time_s) {
  if (response.trajectory.empty()) return nullptr;
  const TimedTrajectoryPoint* result = &response.trajectory.front();
  for (const TimedTrajectoryPoint& point : response.trajectory) {
    if (point.relative_time_s > relative_time_s) break;
    result = &point;
  }
  return result;
}

TimedTrajectoryPoint InterpolatedReferenceAt(const PlanningResponse& response,
                                             double relative_time_s) {
  if (response.trajectory.empty()) return {};
  if (relative_time_s <= response.trajectory.front().relative_time_s) {
    return response.trajectory.front();
  }
  for (size_t index = 1; index < response.trajectory.size(); ++index) {
    const TimedTrajectoryPoint& next = response.trajectory[index];
    if (relative_time_s > next.relative_time_s) continue;
    const TimedTrajectoryPoint& previous = response.trajectory[index - 1];
    const double duration = next.relative_time_s - previous.relative_time_s;
    const double ratio =
        duration <= 1e-9 ? 1.0 : (relative_time_s - previous.relative_time_s) / duration;
    TimedTrajectoryPoint result = previous;
    result.pose.position = Interpolate(previous.pose.position, next.pose.position, ratio);
    result.pose.yaw = NormalizeAngle(
        previous.pose.yaw + ratio * NormalizeAngle(next.pose.yaw - previous.pose.yaw));
    result.speed_mps = previous.speed_mps + ratio * (next.speed_mps - previous.speed_mps);
    result.acceleration_mps2 = previous.acceleration_mps2 +
                               ratio * (next.acceleration_mps2 - previous.acceleration_mps2);
    result.relative_time_s = relative_time_s;
    result.direction = ratio < 1.0 ? previous.direction : next.direction;
    return result;
  }
  return response.trajectory.back();
}
}  // namespace

void AppendEgoHistorySample(std::deque<EgoHistorySample>* history,
                            const EgoHistorySample& sample) {
  if (history == nullptr) return;
  history->push_back(sample);
  const double oldest_time = sample.time_s - kEgoHistoryWindowS;
  while (history->size() > 1 && history->front().time_s < oldest_time) {
    history->pop_front();
  }
}

SimulationRuntime::SimulationRuntime(SimulationScenario scenario) { Reset(std::move(scenario)); }

void SimulationRuntime::Reset(SimulationScenario scenario) {
  scenario_ = std::move(scenario);
  ego_ = scenario_.initial_ego;
  planner_ = std::make_unique<Planner>(scenario_.vehicle, scenario_.planner);
  response_ = {};
  debug_ = {};
  simulation_time_s_ = 0.0;
  trajectory_start_time_s_ = 0.0;
  next_plan_time_s_ = 0.0;
  gear_request_start_time_s_ = 0.0;
  pending_gear_ = DrivingDirection::kUnknown;
  sequence_id_ = 0;
  last_planning_time_ms_ = 0.0;
  running_ = false;
  stop_reason_.clear();
  ego_history_.clear();
  AppendEgoHistorySample(&ego_history_, {simulation_time_s_, ego_});
}

void SimulationRuntime::PlanNow() {
  const PlanningRequest request =
      MakePlanningRequest(scenario_, ego_, simulation_time_s_, ++sequence_id_);
  const auto start = std::chrono::steady_clock::now();
  response_ = planner_->Plan(request, &debug_);
  last_planning_time_ms_ =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  trajectory_start_time_s_ = simulation_time_s_;
  next_plan_time_s_ = simulation_time_s_ + kPlanningPeriodS;
  if (response_.status != PlanningStatus::kOk) {
    running_ = false;
    stop_reason_ = std::string("planner: ") + ToString(response_.status) + ": " + response_.message;
  }
}

void SimulationRuntime::Replan() { PlanNow(); }

void SimulationRuntime::ApplyController(double step_s) {
  if (response_.trajectory.empty()) return;
  const TimedTrajectoryPoint reference = InterpolatedReferenceAt(
      response_, std::max(0.0, simulation_time_s_ - trajectory_start_time_s_));
  const DrivingDirection requested_direction = reference.direction;
  if (requested_direction != ego_.direction) {
    const double deceleration = scenario_.vehicle.max_deceleration_mps2;
    ego_.speed_mps = std::max(0.0, ego_.speed_mps - deceleration * step_s);
    ego_.acceleration_mps2 =
        ego_.speed_mps <= scenario_.planner.gear_shift_stop_speed_mps ? 0.0 : -deceleration;
    if (std::abs(ego_.speed_mps) <= scenario_.planner.gear_shift_stop_speed_mps) {
      if (pending_gear_ != requested_direction) {
        pending_gear_ = requested_direction;
        gear_request_start_time_s_ = simulation_time_s_;
      }
      if (simulation_time_s_ - gear_request_start_time_s_ >= scenario_.planner.gear_shift_dwell_s) {
        ego_.direction = requested_direction;
        ego_.acceleration_mps2 = 0.0;
        pending_gear_ = DrivingDirection::kUnknown;
      }
    }
    return;
  }
  pending_gear_ = DrivingDirection::kUnknown;
  const double target_speed = std::max(0.0, reference.speed_mps);
  const double speed_error = target_speed - ego_.speed_mps;
  const double desired_acceleration = std::clamp(speed_error / step_s,
                                                 -scenario_.vehicle.max_deceleration_mps2,
                                                 scenario_.vehicle.max_acceleration_mps2);
  ego_.acceleration_mps2 = desired_acceleration;
  ego_.speed_mps = std::max(0.0, ego_.speed_mps + desired_acceleration * step_s);

  // Pure pursuit curvature from a point ahead on the planned trajectory, bounded by vehicle limits.
  const double lookahead_s = std::max(0.8, 0.8 + ego_.speed_mps * 0.5);
  const TimedTrajectoryPoint* target = ReferenceAt(
      response_, std::max(0.0, simulation_time_s_ - trajectory_start_time_s_) + lookahead_s / 2.0);
  if (target == nullptr) return;
  const double dx = target->pose.position.x - ego_.pose.position.x;
  const double dy = target->pose.position.y - ego_.pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) return;
  const double target_angle = std::atan2(dy, dx);
  const double alpha = NormalizeAngle(target_angle - ego_.pose.yaw);
  const double curvature = std::clamp(2.0 * std::sin(alpha) / distance,
                                      -scenario_.vehicle.max_curvature_1pm,
                                      scenario_.vehicle.max_curvature_1pm);
  const double signed_speed = ego_.direction == DrivingDirection::kReverse ? -ego_.speed_mps
                                                                            : ego_.speed_mps;
  ego_.pose.yaw = NormalizeAngle(ego_.pose.yaw + signed_speed * curvature * step_s);
  ego_.pose.position.x += signed_speed * std::cos(ego_.pose.yaw) * step_s;
  ego_.pose.position.y += signed_speed * std::sin(ego_.pose.yaw) * step_s;
}

bool SimulationRuntime::HasCollision() const {
  for (const ScenarioObstacle& obstacle : scenario_.obstacles) {
    if (IsVehicleObstacleCollision(ego_.pose, scenario_.vehicle,
                                   SampleObstaclePose(obstacle, simulation_time_s_),
                                   obstacle.length_m, obstacle.width_m)) {
      return true;
    }
  }
  return false;
}

void SimulationRuntime::Step() {
  if (simulation_time_s_ + 1e-9 >= next_plan_time_s_ || response_.trajectory.empty()) PlanNow();
  if (!running_ && !stop_reason_.empty()) return;
  ApplyController(kSimulationStepS);
  simulation_time_s_ += kSimulationStepS;
  AppendEgoHistorySample(&ego_history_, {simulation_time_s_, ego_});
  if (HasCollision()) {
    running_ = false;
    stop_reason_ = "simulation collision";
  }
}

}  // namespace avp::tools
