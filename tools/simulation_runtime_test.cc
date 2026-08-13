#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "tools/simulation_runtime.h"

namespace {
void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(1);
  }
}

void TestKeyframeSamplingAndPrediction() {
  avp::tools::ScenarioObstacle obstacle{"walker", 0.6, 0.6, 1.0, true,
                                         {{0.0, {{0.0, 0.0}, 0.0}, 1.0},
                                          {2.0, {{2.0, 0.0}, 0.0}, 1.0}}};
  Check(std::abs(avp::tools::SampleObstaclePose(obstacle, 1.0).position.x - 1.0) < 1e-9,
        "keyframe pose must interpolate");
  Check(std::abs(avp::tools::SampleObstaclePose(obstacle, 3.0).position.x - 1.0) < 1e-9,
        "looping keyframes must wrap");
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.obstacles = {obstacle};
  const avp::PlanningRequest request = avp::tools::MakePlanningRequest(scenario, scenario.initial_ego, 0.0, 1);
  Check(request.obstacles.size() == 1 && request.obstacles.front().prediction.size() > 2,
        "planner request must contain horizon obstacle prediction");
}

void TestJsonRoundTrip() {
  const auto path = std::filesystem::temp_directory_path() / "avp_qt_scenario_test.json";
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.obstacles.push_back({"static", 1.0, 1.0, 1.0, false, {{0.0, {{2.0, 1.0}, 0.0}, 0.0}}});
  std::string error;
  Check(avp::tools::SaveScenarioJson(scenario, path.string(), &error), "scenario JSON must save");
  avp::tools::SimulationScenario loaded;
  Check(avp::tools::LoadScenarioJson(path.string(), &loaded, &error), "scenario JSON must load");
  Check(loaded.map.lanes.size() == scenario.map.lanes.size() && loaded.obstacles.size() == 1,
        "scenario JSON must preserve content");
  std::filesystem::remove(path);
}

void TestRuntimePlansAndLimitsSpeed() {
  avp::tools::SimulationRuntime runtime(avp::tools::MakeDefaultScenario());
  runtime.Replan();
  Check(!runtime.response().trajectory.empty(), "runtime must request a trajectory");
  runtime.SetRunning(true);
  for (int index = 0; index < 10; ++index) runtime.Step();
  Check(runtime.ego().speed_mps <= runtime.scenario().vehicle.max_speed_mps + 1e-9,
        "runtime must respect maximum vehicle speed");
}
}  // namespace

int main() {
  TestKeyframeSamplingAndPrediction();
  TestJsonRoundTrip();
  TestRuntimePlansAndLimitsSpeed();
  return 0;
}
