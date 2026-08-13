#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "tools/plot_data.h"
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
  const avp::PlanningRequest request =
      avp::tools::MakePlanningRequest(scenario, scenario.initial_ego, 0.0, 1);
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
  Check(runtime.ego_history().size() == 1 && runtime.ego_history().front().time_s == 0.0,
        "runtime reset must create the initial ego history sample");
  runtime.Replan();
  Check(!runtime.response().trajectory.empty(), "runtime must request a trajectory");
  const avp::Vec2 initial_position = runtime.ego().pose.position;
  runtime.SetRunning(true);
  for (int index = 0; index < 50; ++index) runtime.Step();
  Check(runtime.ego().speed_mps <= runtime.scenario().vehicle.max_speed_mps + 1e-9,
        "runtime must respect maximum vehicle speed");
  Check(std::hypot(runtime.ego().pose.position.x - initial_position.x,
                   runtime.ego().pose.position.y - initial_position.y) > 1e-3,
        "running runtime must move the ego vehicle along its trajectory");
  Check(runtime.ego_history().size() > 1 &&
            runtime.ego_history().back().time_s == runtime.simulation_time_s() &&
            runtime.ego_history().back().ego.speed_mps == runtime.ego().speed_mps,
        "ego history must track the simulated state");

  runtime.Reset(avp::tools::MakeDefaultScenario());
  Check(runtime.ego_history().size() == 1 && runtime.ego_history().front().time_s == 0.0,
        "runtime reset must clear prior ego history");
}

void TestEgoHistoryWindow() {
  std::deque<avp::tools::EgoHistorySample> history;
  avp::tools::AppendEgoHistorySample(&history, {0.0, {}});
  avp::tools::AppendEgoHistorySample(&history, {59.0, {}});
  avp::tools::AppendEgoHistorySample(&history, {61.0, {}});
  Check(history.size() == 2 && history.front().time_s == 59.0,
        "ego history must discard samples older than sixty seconds");
  avp::tools::AppendEgoHistorySample(&history, {122.0, {}});
  Check(history.size() == 1 && history.front().time_s == 122.0,
        "ego history must retain at least the current sample");
}

void TestPlanningPlotData() {
  avp::PlanningDebugData debug;
  debug.planning_mode = "LANE_APPROACH";
  debug.cropped_reference_line = {{0.0, 0.0}, {10.0, 0.0}};
  debug.coupling_iterations.push_back(
      {{{{2.0, 1.0}, 0.0, 0.0, 0.0}, {{4.0, -1.0}, 0.0, 0.0, 2.0}}, {}, {}});
  avp::PlanningResponse response;
  response.trajectory = {{{{0.0, 0.0}, 3.1}, 0.1, 0.0, 0.0, 0.0,
                          avp::DrivingDirection::kDrive},
                         {{{1.0, 0.0}, -3.1}, -0.2, 0.0, 0.0, 1.0,
                          avp::DrivingDirection::kDrive}};
  const avp::tools::PlanningPlotData plots =
      avp::tools::BuildPlanningPlotData(debug, response);
  Check(plots.sl_path.size() == 2 && std::abs(plots.sl_path[0].x - 2.0) < 1e-9 &&
            std::abs(plots.sl_path[0].y - 1.0) < 1e-9 &&
            std::abs(plots.sl_path[1].y + 1.0) < 1e-9,
        "S-L projection must preserve reference progress and lateral sign");
  Check(plots.st_path.size() == 2 && std::abs(plots.st_path.back().y - 1.0) < 1e-9 &&
            plots.curvature_by_s.back().y == -0.2,
        "trajectory plots must use accumulated arc length");
  Check(plots.yaw_by_s.back().y > plots.yaw_by_s.front().y &&
            plots.yaw_by_s.back().y - plots.yaw_by_s.front().y < 10.0,
        "S-yaw must unwrap the pi boundary without a false jump");

  debug.planning_mode = "OPEN_SPACE_PARKING";
  const avp::tools::PlanningPlotData parking =
      avp::tools::BuildPlanningPlotData(debug, response);
  Check(parking.sl_path.empty() && !parking.sl_empty_message.empty() &&
            !parking.st_path.empty(),
        "parking must suppress S-L while retaining trajectory plots");
}

void TestDriveAndParkScenarioLoads() {
  const std::filesystem::path path =
      std::filesystem::path(AVP_SOURCE_DIR) / "tools/scenarios/drive_and_park_dynamic.json";
  avp::tools::SimulationScenario scenario;
  std::string error;
  Check(avp::tools::LoadScenarioJson(path.string(), &scenario, &error),
        "dynamic drive-and-park scenario must load");
  Check(scenario.map.lanes.size() >= 2 && scenario.map.parking_spots.size() == 1 &&
            scenario.obstacles.size() >= 2,
        "dynamic drive-and-park scenario must contain a route, parking spot, and obstacles");
  avp::tools::SimulationRuntime runtime(std::move(scenario));
  runtime.Replan();
  Check(runtime.response().status == avp::PlanningStatus::kOk &&
            !runtime.response().trajectory.empty(),
        "dynamic drive-and-park scenario must have a valid initial plan");
}
}  // namespace

int main() {
  TestKeyframeSamplingAndPrediction();
  TestJsonRoundTrip();
  TestRuntimePlansAndLimitsSpeed();
  TestEgoHistoryWindow();
  TestPlanningPlotData();
  TestDriveAndParkScenarioLoads();
  return 0;
}
