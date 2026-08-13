#include "tools/simulation_runtime.h"

#include <algorithm>
#include <chrono>

namespace avp::tools {
namespace {
constexpr double kSimulationStepS = 0.02;
constexpr double kPlanningPeriodS = 0.2;

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
  sequence_id_ = 0;
  last_planning_time_ms_ = 0.0;
  running_ = false;
  stop_reason_.clear();
  ego_history_.clear();
  AppendEgoHistorySample(&ego_history_, {simulation_time_s_, ego_});
  PlanNow();
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
  if (response_.status == PlanningStatus::kOk) {
    if (stop_reason_.rfind("planner: ", 0) == 0) stop_reason_.clear();
    return;
  }
  stop_reason_ = std::string("planner: ") + ToString(response_.status) + ": " + response_.message;
}

void SimulationRuntime::FollowTrajectory(double step_s) {
  if (response_.trajectory.empty()) return;
  const TimedTrajectoryPoint reference = InterpolatedReferenceAt(
      response_, std::max(0.0, simulation_time_s_ + step_s - trajectory_start_time_s_));
  ego_.pose = reference.pose;
  ego_.speed_mps = reference.speed_mps;
  ego_.acceleration_mps2 = reference.acceleration_mps2;
  ego_.direction = reference.direction;
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

void SimulationRuntime::Tick() {
  if (simulation_time_s_ + 1e-9 >= next_plan_time_s_) PlanNow();
  FollowTrajectory(kSimulationStepS);
  simulation_time_s_ += kSimulationStepS;
  AppendEgoHistorySample(&ego_history_, {simulation_time_s_, ego_});
  if (HasCollision()) {
    stop_reason_ = "simulation collision";
  } else if (stop_reason_ == "simulation collision") {
    stop_reason_.clear();
  }
}

}  // namespace avp::tools
