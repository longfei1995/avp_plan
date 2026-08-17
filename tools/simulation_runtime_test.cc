#include "tools/simulation_runtime.h"

#include <QApplication>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "tools/plot_data.h"
#include "tools/scene_rendering.h"

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
    result.acceleration_mps2 =
        previous.acceleration_mps2 + ratio * (next.acceleration_mps2 - previous.acceleration_mps2);
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
  avp::tools::ScenarioObstacle obstacle{
      "walker", 0.6,  0.6,
      1.0,      true, {{0.0, {{0.0, 0.0}, 0.0}, 1.0}, {2.0, {{2.0, 0.0}, 0.0}, 1.0}}};
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
  CheckNear(loaded.planner.max_parking_speed_mps, scenario.planner.max_parking_speed_mps,
            "scenario JSON must preserve the general parking speed limit");
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

void TestCruiseAndEndpointLiveness() {
  avp::tools::SimulationScenario cruise_scenario = avp::tools::MakeDefaultScenario();
  cruise_scenario.map.lanes = {{"long", {{0.0, 0.0}, {50.0, 0.0}}, {}, false}};
  cruise_scenario.map.parking_spots = {{"P", {{50.0, 0.0}, 0.0}, {{51.0, 0.0}, 0.0}}};
  cruise_scenario.target_parking_spot_id = "P";
  avp::tools::SimulationRuntime cruise_runtime(std::move(cruise_scenario));
  cruise_runtime.SetRunning(true);
  while (cruise_runtime.simulation_time_s() < 4.0 - 1e-9) cruise_runtime.Tick();
  Check(
      cruise_runtime.response().status == avp::PlanningStatus::kOk &&
          cruise_runtime.ego().speed_mps >= 2.7 &&
          cruise_runtime.ego().speed_mps <= cruise_runtime.scenario().vehicle.max_speed_mps + 1e-9,
      "rolling replanning should reach ninety percent of maximum cruise speed in four seconds");

  avp::tools::SimulationRuntime endpoint_runtime(avp::tools::MakeDefaultScenario());
  endpoint_runtime.SetRunning(true);
  double current_creep_s = 0.0;
  double longest_creep_s = 0.0;
  bool entered_parking = false;
  while (endpoint_runtime.simulation_time_s() < 10.0 - 1e-9) {
    endpoint_runtime.Tick();
    const bool away_from_entry =
        avp::Distance(endpoint_runtime.ego().pose.position,
                      endpoint_runtime.scenario().map.parking_spots.front().entry_pose.position) >
        0.55;
    const double speed = endpoint_runtime.ego().speed_mps;
    if (endpoint_runtime.debug().planning_mode == "LANE_APPROACH" && away_from_entry &&
        speed > 0.0 && speed < endpoint_runtime.scenario().planner.gear_shift_stop_speed_mps) {
      current_creep_s += 0.02;
      longest_creep_s = std::max(longest_creep_s, current_creep_s);
    } else {
      current_creep_s = 0.0;
    }
    if (endpoint_runtime.debug().planning_mode == "OPEN_SPACE_PARKING") {
      entered_parking = true;
      for (const avp::TimedTrajectoryPoint& point : endpoint_runtime.response().trajectory) {
        Check(point.speed_mps <= endpoint_runtime.scenario().planner.max_parking_speed_mps + 1e-9,
              "forward open-space parking must respect the general parking speed limit");
      }
    }
  }
  Check(entered_parking, "default rolling scenario must leave lane approach within ten seconds");
  Check(longest_creep_s <= 1.0 + 1e-9,
        "lane approach must not sustain sub-threshold creep away from the parking entry");
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
  scenario.obstacles = {{"wall", 1.0, 6.0, 1.0, false, {{0.0, {{5.0, 0.0}, 0.0}, 0.0}}}};
  avp::tools::SimulationRuntime runtime(std::move(scenario));

  Check(runtime.response().status == avp::PlanningStatus::kNoSafeTrajectory,
        "wall must block every S-L lattice path and produce a safe-stop fallback");
  Check(!runtime.stop_reason().empty(), "planning fallback must remain visible as a warning");
  for (const avp::TimedTrajectoryPoint& point : runtime.response().trajectory) {
    CheckNear(point.speed_mps, 0.0, "stationary fallback must keep zero speed");
    CheckNear(point.acceleration_mps2, 0.0,
              "stationary fallback must not retain emergency deceleration");
  }

  runtime.SetRunning(true);
  for (int index = 0; index < 20; ++index) runtime.Tick();
  CheckNear(runtime.simulation_time_s(), 0.4,
            "safe fallback must keep advancing time through repeated planning failures");
  Check(runtime.running(), "safe fallback must never change the user-selected Run state");
  Check(runtime.response().header.sequence_id == 2,
        "fallback execution must retain the five-hertz rolling replan cadence");
}

void TestMovingFallbackEndsAtRest() {
  avp::tools::SimulationScenario scenario = avp::tools::MakeDefaultScenario();
  scenario.initial_ego.speed_mps = 1.0;
  scenario.obstacles = {{"wall", 1.0, 6.0, 1.0, false, {{0.0, {{5.0, 0.0}, 0.0}, 0.0}}}};
  avp::tools::SimulationRuntime runtime(scenario);

  Check(runtime.response().status == avp::PlanningStatus::kNoSafeTrajectory,
        "moving blocked ego must receive a safe-stop fallback");
  const std::vector<avp::TimedTrajectoryPoint>& trajectory = runtime.response().trajectory;
  Check(trajectory.size() >= 2, "moving fallback must contain a stopping endpoint");
  CheckNear(trajectory.back().speed_mps, 0.0, "moving fallback must end at zero speed");
  CheckNear(trajectory.back().acceleration_mps2, 0.0,
            "moving fallback must clear acceleration once stopped");

  const double braking_distance = scenario.initial_ego.speed_mps * scenario.initial_ego.speed_mps /
                                  (2.0 * scenario.vehicle.max_deceleration_mps2);
  double previous_distance = 0.0;
  double previous_speed = trajectory.front().speed_mps;
  for (const avp::TimedTrajectoryPoint& point : trajectory) {
    const double distance = point.pose.position.x - scenario.initial_ego.pose.position.x;
    Check(distance + 1e-9 >= previous_distance, "moving fallback distance must be monotonic");
    Check(distance <= braking_distance + 1e-9,
          "moving fallback must not pass its physical braking endpoint");
    Check(point.speed_mps <= previous_speed + 1e-9, "moving fallback speed must be monotonic");
    previous_distance = distance;
    previous_speed = point.speed_mps;
  }
  CheckNear(previous_distance, braking_distance,
            "moving fallback must finish at the physical braking endpoint");
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
  scenario.obstacles = {{"overlap", 1.0, 1.0, 1.0, false, {{0.0, scenario.initial_ego.pose, 0.0}}}};
  avp::tools::SimulationRuntime runtime(std::move(scenario));
  runtime.SetRunning(true);

  runtime.Tick();
  Check(runtime.running(), "simulation collision must not change the user-selected Run state");
  CheckNear(runtime.simulation_time_s(), 0.02,
            "simulation collision must still advance simulation time");
  Check(runtime.stop_reason() == "simulation collision",
        "simulation collision must remain visible as a warning");
}

void TestSceneRenderingMatchesCollisionGeometry() {
  avp::VehicleConfig vehicle;
  vehicle.length_m = 4.8;
  vehicle.width_m = 2.0;
  vehicle.safety_margin_m = 0.35;
  const avp::Pose2d pose{{3.0, -1.0}, 0.0};

  const QPolygonF body = avp::tools::OrientedBoxPolygon(pose, vehicle.length_m, vehicle.width_m);
  const QPolygonF safety = avp::tools::VehicleSafetyPolygon(pose, vehicle);
  CheckNear(body.boundingRect().width(), vehicle.length_m,
            "rendered vehicle body must preserve configured length");
  CheckNear(body.boundingRect().height(), vehicle.width_m,
            "rendered vehicle body must preserve configured width");
  CheckNear(safety.boundingRect().width(), vehicle.length_m + 2.0 * vehicle.safety_margin_m,
            "rendered safety envelope must expand both longitudinal sides");
  CheckNear(safety.boundingRect().height(), vehicle.width_m + 2.0 * vehicle.safety_margin_m,
            "rendered safety envelope must expand both lateral sides");

  const QPen body_pen = avp::tools::ThinCosmeticPen(Qt::blue);
  const QPen safety_pen = avp::tools::ThinCosmeticPen(Qt::blue, Qt::DashLine);
  Check(body_pen.isCosmetic() && std::abs(body_pen.widthF() - 1.0) < 1e-9,
        "vehicle and obstacle outlines must remain one pixel wide while zooming");
  Check(safety_pen.isCosmetic() && safety_pen.style() == Qt::DashLine,
        "safety envelope must use a cosmetic dashed outline");

  QGraphicsScene scene;
  const avp::Vec2 entry_position{7.0, -3.0};
  QGraphicsEllipseItem* entry = avp::tools::AddParkingEntryMarker(&scene, entry_position);
  Check(entry != nullptr, "parking entry marker must be added to a valid scene");
  CheckNear(entry->rect().width(), avp::tools::kParkingEntryMarkerDiameterPx,
            "parking entry marker must be eight pixels wide");
  CheckNear(entry->rect().height(), avp::tools::kParkingEntryMarkerDiameterPx,
            "parking entry marker must be eight pixels high");
  CheckNear(entry->rect().center().x(), 0.0,
            "parking entry marker must be centered on its local origin");
  CheckNear(entry->rect().center().y(), 0.0,
            "parking entry marker must be centered on its local origin");
  CheckNear(entry->pos().x(), entry_position.x,
            "parking entry marker must preserve the scene x coordinate");
  CheckNear(entry->pos().y(), -entry_position.y,
            "parking entry marker must convert the scene y coordinate");
  Check(entry->flags().testFlag(QGraphicsItem::ItemIgnoresTransformations),
        "parking entry marker must remain fixed-size while zooming");
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

void TestResponseTrajectoryTableData() {
  Check(avp::tools::BuildResponseTrajectoryRows({}).empty(),
        "empty planning response must produce an empty trajectory table");

  avp::PlanningResponse response;
  response.trajectory = {
      {{{1.0, 2.0}, 0.3}, 0.1, 2.0, -0.5, 0.4, avp::DrivingDirection::kDrive},
      {{{3.0, 4.0}, -0.2}, -0.1, 1.0, 0.25, 0.8, avp::DrivingDirection::kReverse},
      {{{5.0, 6.0}, 0.0}, 0.0, 0.0, 0.0, 1.2, avp::DrivingDirection::kUnknown}};
  const std::vector<avp::tools::ResponseTrajectoryRow> rows =
      avp::tools::BuildResponseTrajectoryRows(response);
  Check(rows.size() == 3, "every response trajectory point must produce one table row");
  Check(rows[0].index == 0 && rows[0].gear == "D" && rows[1].index == 1 && rows[1].gear == "R" &&
            rows[2].index == 2 && rows[2].gear == "N",
        "trajectory table must preserve order and convert every gear value");
  CheckNear(rows[0].relative_time_s, 0.4, "trajectory table must preserve relative time");
  CheckNear(rows[0].x_m, 1.0, "trajectory table must preserve x");
  CheckNear(rows[0].y_m, 2.0, "trajectory table must preserve y");
  CheckNear(rows[0].yaw_rad, 0.3, "trajectory table must preserve yaw");
  CheckNear(rows[0].curvature_1pm, 0.1, "trajectory table must preserve curvature");
  CheckNear(rows[0].speed_mps, 2.0, "trajectory table must preserve speed");
  CheckNear(rows[0].acceleration_mps2, -0.5, "trajectory table must preserve acceleration");
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
            runtime.response().status == avp::PlanningStatus::kOk && runtime.stop_reason().empty(),
        "dynamic drive-and-park scenario must remain feasible after its first rolling replan");

  bool observed_temporary_fallback = false;
  const double position_before_crossing = runtime.ego().pose.position.x;
  while (runtime.simulation_time_s() < 2.2) {
    runtime.Tick();
    if (runtime.response().status == avp::PlanningStatus::kNoSafeTrajectory) {
      observed_temporary_fallback = true;
    }
  }
  Check(observed_temporary_fallback,
        "crossing pedestrian scenario must exercise the temporary safe-stop fallback");
  Check(runtime.response().status == avp::PlanningStatus::kOk && runtime.stop_reason().empty(),
        "planner must recover after the non-looping pedestrian clears the lane");

  const double recovery_position = runtime.ego().pose.position.x;
  for (int index = 0; index < 20; ++index) runtime.Tick();
  Check(runtime.response().status == avp::PlanningStatus::kOk && runtime.ego().speed_mps > 0.0 &&
            runtime.ego().pose.position.x > recovery_position + 1e-3 &&
            runtime.ego().pose.position.x > position_before_crossing + 1e-3,
        "ego must resume forward motion after the pedestrian clears the lane");
}

void TestStaticObstacleAvoidanceScenarioLoads() {
  const std::filesystem::path path =
      std::filesystem::path(AVP_SOURCE_DIR) / "tools/scenarios/static_obstacle_avoidance_100m.json";
  avp::tools::SimulationScenario scenario;
  std::string error;
  Check(avp::tools::LoadScenarioJson(path.string(), &scenario, &error),
        "static-obstacle avoidance scenario must load");
  Check(scenario.map.lanes.size() == 2 && scenario.map.parking_spots.size() == 1 &&
            scenario.obstacles.size() == 3,
        "static-obstacle avoidance scenario must provide two lanes, a drive goal, and obstacles");
  Check(std::all_of(scenario.obstacles.begin(), scenario.obstacles.end(),
                    [](const avp::tools::ScenarioObstacle& obstacle) {
                      return obstacle.keyframes.size() == 1 &&
                             obstacle.keyframes.front().speed_mps == 0.0;
                    }),
        "static-obstacle avoidance scenario must contain only static obstacles");

  avp::tools::SimulationRuntime runtime(std::move(scenario));
  Check(runtime.response().status == avp::PlanningStatus::kOk && runtime.stop_reason().empty(),
        "static-obstacle avoidance scenario must have a feasible initial plan");
  const std::vector<avp::PathPoint>& initial_path =
      runtime.debug().coupling_iterations.back().local_path;
  Check(std::any_of(initial_path.begin(), initial_path.end(),
                    [](const avp::PathPoint& point) {
                      return point.position.x > 19.0 && point.position.x < 24.0 &&
                             point.position.y < -0.2;
                    }),
        "static-obstacle avoidance scenario must initially detour around the first right-lane "
        "barrier");
}
}  // namespace

int main(int argc, char* argv[]) {
  qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication application(argc, argv);
  TestKeyframeSamplingAndPrediction();
  TestJsonRoundTrip();
  TestRuntimePlansAndLimitsSpeed();
  TestPerfectTrajectoryTrackingAndRollingReplan();
  TestCruiseAndEndpointLiveness();
  TestPerfectTrackingPreservesGearShiftDwell();
  TestSafeFallbackNeverPausesSimulation();
  TestMovingFallbackEndsAtRest();
  TestFatalPlanningFailureNeverPausesSimulation();
  TestCollisionNeverPausesSimulation();
  TestSceneRenderingMatchesCollisionGeometry();
  TestEgoHistoryWindow();
  TestResponseTrajectoryTableData();
  TestDriveAndParkScenarioLoads();
  TestStaticObstacleAvoidanceScenarioLoads();
  return 0;
}
