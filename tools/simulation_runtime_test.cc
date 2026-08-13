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

void CheckNear(double actual, double expected, const char* message) {
  Check(std::abs(actual - expected) <= 1e-9, message);
}

avp::TimedTrajectoryPoint SampleExpectedTrajectory(const avp::PlanningResponse& response,
                                                   double relative_time_s) {
  Check(!response.trajectory.empty(), "expected trajectory must not be empty");
  if (relative_time_s <= response.trajectory.front().relative_time_s) {
    return response.trajectory.front();
  }
  for (size_t index = 1; index < response.trajectory.size(); ++index) {
    const avp::TimedTrajectoryPoint& next = response.trajectory[index];
    if (relative_time_s > next.relative_time_s) continue;
    const avp::TimedTrajectoryPoint& previous = response.trajectory[index - 1];
    const double duration = next.relative_time_s - previous.relative_time_s;
    const double ratio =
        duration <= 1e-9 ? 1.0 : (relative_time_s - previous.relative_time_s) / duration;
    avp::TimedTrajectoryPoint result = previous;
    result.pose.position = avp::Interpolate(previous.pose.position, next.pose.position, ratio);
    result.pose.yaw = avp::NormalizeAngle(
        previous.pose.yaw + ratio * avp::NormalizeAngle(next.pose.yaw - previous.pose.yaw));
    result.speed_mps = previous.speed_mps + ratio * (next.speed_mps - previous.speed_mps);
    result.acceleration_mps2 = previous.acceleration_mps2 +
                               ratio * (next.acceleration_mps2 - previous.acceleration_mps2);
    result.relative_time_s = relative_time_s;
    result.direction = ratio < 1.0 ? previous.direction : next.direction;
    return result;
  }
  return response.trajectory.back();
}

void CheckEgoMatches(const avp::EgoState& ego, const avp::TimedTrajectoryPoint& expected,
                     const char* message) {
  CheckNear(ego.pose.position.x, expected.pose.position.x, message);
  CheckNear(ego.pose.position.y, expected.pose.position.y, message);
  CheckNear(avp::NormalizeAngle(ego.pose.yaw - expected.pose.yaw), 0.0, message);
  CheckNear(ego.speed_mps, expected.speed_mps, message);
  CheckNear(ego.acceleration_mps2, expected.acceleration_mps2, message);
  Check(ego.direction == expected.direction, message);
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
  Check(!runtime.response().trajectory.empty(), "runtime must request a trajectory");
  const avp::Vec2 initial_position = runtime.ego().pose.position;
  runtime.SetRunning(true);
  for (int index = 0; index < 50; ++index) runtime.Tick();
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
  CheckNear(runtime.simulation_time_s(), 0.0, "runtime reset must restore initial simulation time");
  CheckNear(runtime.ego().pose.position.x, initial_position.x,
            "runtime reset must restore the initial ego x position");
  CheckNear(runtime.ego().pose.position.y, initial_position.y,
            "runtime reset must restore the initial ego y position");
  Check(!runtime.running(), "runtime reset must restore the initially paused state");
  Check(runtime.response().status == avp::PlanningStatus::kOk &&
            runtime.response().header.sequence_id == 1,
        "runtime reset must recreate the initial planning response and planner sequence");
}

void TestPerfectTrajectoryTrackingAndRollingReplan() {
  avp::tools::SimulationRuntime runtime(avp::tools::MakeDefaultScenario());
  Check(runtime.response().status == avp::PlanningStatus::kOk &&
            runtime.response().trajectory.size() >= 2,
        "perfect tracking test requires a valid timed trajectory");
  const uint64_t first_sequence_id = runtime.response().header.sequence_id;
  const avp::TimedTrajectoryPoint first_expected =
      SampleExpectedTrajectory(runtime.response(), 0.02);

  runtime.Tick();
  CheckNear(runtime.simulation_time_s(), 0.02,
            "one simulation step must advance exactly twenty milliseconds");
  CheckEgoMatches(runtime.ego(), first_expected,
                  "perfect tracking must use the trajectory state at the next simulation time");

  for (int index = 1; index < 10; ++index) runtime.Tick();
  CheckNear(runtime.simulation_time_s(), 0.2,
            "ten simulation steps must reach one planning period");
  Check(runtime.response().header.sequence_id == first_sequence_id,
        "replanning must not occur before the next step starts at the planning boundary");
  const avp::EgoState replan_anchor = runtime.ego();

  runtime.Tick();
  Check(runtime.response().header.sequence_id == first_sequence_id + 1,
        "the first step at the planning boundary must trigger one rolling replan");
  CheckNear(runtime.response().trajectory.front().pose.position.x, replan_anchor.pose.position.x,
            "rolling replan trajectory must remain anchored at the current ego x position");
  CheckNear(runtime.response().trajectory.front().pose.position.y, replan_anchor.pose.position.y,
            "rolling replan trajectory must remain anchored at the current ego y position");
  CheckEgoMatches(runtime.ego(), SampleExpectedTrajectory(runtime.response(), 0.02),
                  "perfect tracking must continue from the replanned trajectory without lag");
}

void TestPerfectTrackingPreservesGearShiftDwell() {
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.map.lanes = {{"entry", {{0.0, 0.0}, {10.0, 0.0}}, {}, false}};
  scenario.map.parking_spots = {{"P", {{10.0, 0.0}, 0.0}, {{9.0, 0.0}, 0.0}}};
  scenario.initial_ego.pose = {{10.0, 0.0}, 0.0};
  scenario.initial_ego.direction = avp::DrivingDirection::kDrive;
  scenario.target_parking_spot_id = "P";
  const double gear_shift_dwell_s = scenario.planner.gear_shift_dwell_s;
  avp::tools::SimulationRuntime runtime(std::move(scenario));

  Check(runtime.response().status == avp::PlanningStatus::kOk &&
            runtime.response().message == "waiting for safe gear shift",
        "reverse parking must begin with a stationary gear-shift trajectory");
  const avp::Pose2d shift_pose = runtime.ego().pose;
  runtime.Tick();
  Check(runtime.ego().direction == avp::DrivingDirection::kReverse,
        "perfect tracking must apply the requested trajectory direction directly");

  for (int index = 1; index < 50; ++index) runtime.Tick();
  CheckNear(runtime.simulation_time_s(), gear_shift_dwell_s,
            "gear-shift regression must reach the configured dwell boundary");
  Check(runtime.response().message == "waiting for safe gear shift",
        "parking planner must retain the stationary trajectory for the full dwell");
  CheckNear(runtime.ego().pose.position.x, shift_pose.position.x,
            "ego x position must remain stationary during gear-shift dwell");
  CheckNear(runtime.ego().pose.position.y, shift_pose.position.y,
            "ego y position must remain stationary during gear-shift dwell");
  CheckNear(runtime.ego().speed_mps, 0.0, "ego speed must remain zero during gear-shift dwell");

  runtime.Tick();
  Check(runtime.response().status == avp::PlanningStatus::kOk &&
            runtime.response().message == "open-space parking trajectory generated",
        "parking motion may start once dwell time and direction feedback are satisfied");
}

void TestSafeFallbackNeverPausesSimulation() {
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.obstacles = {
      {"wall", 1.0, 6.0, 1.0, false, {{0.0, {{5.0, 0.0}, 0.0}, 0.0}}}};
  avp::tools::SimulationRuntime runtime(std::move(scenario));

  Check(runtime.response().status == avp::PlanningStatus::kNoSafeTrajectory,
        "wall must block every S-L lattice path and produce a safe-stop fallback");
  Check(!runtime.stop_reason().empty(), "planning fallback must remain visible as a warning");

  runtime.SetRunning(true);
  for (int index = 0; index < 20; ++index) runtime.Tick();
  CheckNear(runtime.simulation_time_s(), 0.4,
            "safe fallback must keep advancing time through repeated planning failures");
  Check(runtime.running(), "safe fallback must never change the user-selected Run state");
  Check(runtime.response().header.sequence_id == 2,
        "fallback execution must retain the five-hertz rolling replan cadence");
}

void TestFatalPlanningFailureNeverPausesSimulation() {
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.target_parking_spot_id = "missing";
  avp::tools::SimulationRuntime runtime(std::move(scenario));
  runtime.SetRunning(true);

  Check(runtime.response().status != avp::PlanningStatus::kOk &&
            runtime.response().status != avp::PlanningStatus::kNoSafeTrajectory,
        "missing target must produce a fatal planning status");
  Check(runtime.running() && !runtime.stop_reason().empty(),
        "fatal planning failure must report a warning without changing the Run state");
  runtime.Tick();
  CheckNear(runtime.simulation_time_s(), 0.02,
            "fatal planning failure with no trajectory must still advance simulation time");
  Check(runtime.response().header.sequence_id == 1,
        "an empty failed response must not increase replanning beyond five hertz");
}

void TestCollisionNeverPausesSimulation() {
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.obstacles = {
      {"overlap", 1.0, 1.0, 1.0, false, {{0.0, scenario.initial_ego.pose, 0.0}}}};
  avp::tools::SimulationRuntime runtime(std::move(scenario));
  runtime.SetRunning(true);

  runtime.Tick();
  Check(runtime.running(), "simulation collision must not change the user-selected Run state");
  CheckNear(runtime.simulation_time_s(), 0.02,
            "simulation collision must still advance simulation time");
  Check(runtime.stop_reason() == "simulation collision",
        "simulation collision must remain visible as a warning");
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
            !scenario.obstacles.empty(),
        "dynamic drive-and-park scenario must contain a route, parking spot, and obstacles");
  avp::tools::SimulationRuntime runtime(std::move(scenario));
  Check(runtime.response().status == avp::PlanningStatus::kOk &&
            !runtime.response().trajectory.empty(),
        "dynamic drive-and-park scenario must have a valid initial plan");
  runtime.SetRunning(true);
  for (int index = 0; index < 11; ++index) runtime.Tick();
  Check(runtime.simulation_time_s() > 0.2 &&
            runtime.response().status == avp::PlanningStatus::kOk &&
            runtime.stop_reason().empty(),
        "dynamic drive-and-park scenario must remain feasible after its first rolling replan");
}
}  // namespace

int main() {
  TestKeyframeSamplingAndPrediction();
  TestJsonRoundTrip();
  TestRuntimePlansAndLimitsSpeed();
  TestPerfectTrajectoryTrackingAndRollingReplan();
  TestPerfectTrackingPreservesGearShiftDwell();
  TestSafeFallbackNeverPausesSimulation();
  TestFatalPlanningFailureNeverPausesSimulation();
  TestCollisionNeverPausesSimulation();
  TestEgoHistoryWindow();
  TestPlanningPlotData();
  TestDriveAndParkScenarioLoads();
  return 0;
}
