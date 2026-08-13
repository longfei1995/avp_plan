#ifndef AVP_TOOLS_SIMULATION_RUNTIME_H_
#define AVP_TOOLS_SIMULATION_RUNTIME_H_

#include <memory>
#include <string>

#include "planning/planner.h"
#include "tools/scenario.h"

namespace avp::tools {

class SimulationRuntime {
 public:
  explicit SimulationRuntime(SimulationScenario scenario = MakeDefaultScenario());

  void Reset(SimulationScenario scenario);
  void SetRunning(bool running) { running_ = running; }
  bool running() const { return running_; }
  void Step();
  void Replan();

  const SimulationScenario& scenario() const { return scenario_; }
  const EgoState& ego() const { return ego_; }
  double simulation_time_s() const { return simulation_time_s_; }
  const PlanningResponse& response() const { return response_; }
  const PlanningDebugData& debug() const { return debug_; }
  double last_planning_time_ms() const { return last_planning_time_ms_; }
  const std::string& stop_reason() const { return stop_reason_; }

 private:
  void PlanNow();
  void ApplyController(double step_s);
  bool HasCollision() const;

  SimulationScenario scenario_;
  EgoState ego_;
  std::unique_ptr<Planner> planner_;
  PlanningResponse response_;
  PlanningDebugData debug_;
  double simulation_time_s_ = 0.0;
  double trajectory_start_time_s_ = 0.0;
  double next_plan_time_s_ = 0.0;
  double gear_request_start_time_s_ = 0.0;
  DrivingDirection pending_gear_ = DrivingDirection::kUnknown;
  uint64_t sequence_id_ = 0;
  double last_planning_time_ms_ = 0.0;
  bool running_ = false;
  std::string stop_reason_;
};

}  // namespace avp::tools

#endif  // AVP_TOOLS_SIMULATION_RUNTIME_H_
