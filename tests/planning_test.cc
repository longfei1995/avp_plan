#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>

#include "interfaces/adapters.h"
#include "local/local_planner.h"
#include "map/global_planner.h"
#include "open_space/hybrid_a_star.h"
#include "planning/planner.h"
#include "speed/speed_planner.h"

#include "collision_test_cases.h"

namespace {
void Check(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
bool NearlyEqual(double first, double second, double tolerance = 1e-6) {
  return std::abs(first - second) <= tolerance;
}
bool HasDiagnostic(const avp::PlanningResponse& response, const std::string& diagnostic) {
  for (const std::string& item : response.diagnostics) {
    if (item == diagnostic) {
      return true;
    }
  }
  return false;
}
double DiagnosticValue(const avp::PlanningResponse& response, const std::string& prefix) {
  for (const std::string& item : response.diagnostics) {
    if (item.starts_with(prefix)) return std::stod(item.substr(prefix.size()));
  }
  return std::numeric_limits<double>::quiet_NaN();
}
double SumSquaredJerk(const std::vector<avp::SpeedPoint>& profile, double time_step_s) {
  double result = 0.0;
  for (size_t index = 1; index < profile.size(); ++index) {
    const double jerk =
        (profile[index].acceleration_mps2 - profile[index - 1].acceleration_mps2) /
        time_step_s;
    result += jerk * jerk;
  }
  return result;
}
void CheckPosition(const avp::Vec2& actual, const avp::Vec2& expected, const char* message) {
  Check(NearlyEqual(actual.x, expected.x) && NearlyEqual(actual.y, expected.y), message);
}
avp::PlanningRequest MakeRequest() {
  avp::PlanningRequest request;
  request.header = {"map", 1000000000, 1};
  request.ego.pose = {{0.0, 0.0}, 0.0};
  request.map.lanes.push_back({"entry", {{0.0, 0.0}, {5.0, 0.0}}, {"parking_lane"}, false});
  request.map.lanes.push_back({"parking_lane", {{5.0, 0.0}, {10.0, 0.0}}, {}, false});
  request.map.parking_spots.push_back({"P1", {{10.0, 0.0}, 0.0}, {{11.0, 0.0}, 0.0}});
  request.target_parking_spot_id = "P1";
  return request;
}

avp::PlanningFrame MakeGlobalFrame(const avp::MapSnapshot* map, const avp::Pose2d& ego_pose,
                                   const std::string& target_parking_spot_id) {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.ego.pose = ego_pose;
  frame.map = map;
  frame.target_parking_spot_id = target_parking_spot_id;
  return frame;
}

void TestGlobalRouteUsesLaneProjections() {
  avp::MapSnapshot map;
  map.lanes.push_back({"main", {{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}}, {}, false});
  map.parking_spots.push_back({"P", {{15.0, 0.0}, 0.0}, {{15.0, 0.0}, 0.0}});
  avp::GlobalPlanner planner;
  avp::GlobalRoute route;
  std::string error;
  const avp::PlanningFrame frame = MakeGlobalFrame(&map, {{5.0, 0.0}, 0.0}, "P");
  Check(planner.Plan(frame, &route, &error), "mid-lane route should be planned");
  Check(route.lane_ids.size() == 1 && route.lane_ids.front() == "main",
        "same-lane route should keep one lane id");
  CheckPosition(route.reference_line.front(), {5.0, 0.0},
                "reference line must begin at ego projection");
  CheckPosition(route.reference_line.back(), {15.0, 0.0},
                "reference line must end at parking entry");
  for (const avp::Vec2& point : route.reference_line) {
    Check(point.x <= 15.0 + 1e-6, "reference line must not pass a mid-lane entry");
  }
}

void TestSameLaneEntryBehindEgoIsUnreachable() {
  avp::MapSnapshot map;
  map.lanes.push_back({"main", {{0.0, 0.0}, {20.0, 0.0}}, {}, false});
  map.parking_spots.push_back({"P", {{5.0, 0.0}, 0.0}, {{5.0, 0.0}, 0.0}});
  avp::GlobalPlanner planner;
  avp::GlobalRoute route;
  std::string error;
  const avp::PlanningFrame frame = MakeGlobalFrame(&map, {{12.0, 0.0}, 0.0}, "P");
  Check(!planner.Plan(frame, &route, &error), "entry behind ego must not reverse along a lane");
}

void TestLaneProjectionMatching() {
  avp::MapSnapshot sparse_map;
  sparse_map.lanes.push_back({"sparse", {{0.0, 0.0}, {100.0, 0.0}}, {}, false});
  sparse_map.parking_spots.push_back({"P", {{80.0, 0.0}, 0.0}, {{80.0, 0.0}, 0.0}});
  avp::GlobalPlanner planner;
  avp::GlobalRoute route;
  std::string error;
  Check(planner.Plan(MakeGlobalFrame(&sparse_map, {{50.0, 0.0}, 0.0}, "P"), &route, &error),
        "projection must match the middle of a sparse segment");
  CheckPosition(route.reference_line.front(), {50.0, 0.0},
                "sparse lane route must start at segment projection");

  avp::MapSnapshot opposite_map;
  opposite_map.lanes.push_back({"westbound", {{100.0, 0.0}, {0.0, 0.0}}, {}, false});
  opposite_map.parking_spots.push_back(
      {"P", {{20.0, 0.0}, avp::kPi}, {{20.0, 0.0}, avp::kPi}});
  Check(!planner.Plan(MakeGlobalFrame(&opposite_map, {{50.0, 0.0}, 0.0}, "P"), &route, &error),
        "opposite-direction lane must not match ego heading");

  avp::MapSnapshot distant_map;
  distant_map.lanes.push_back({"main", {{0.0, 0.0}, {20.0, 0.0}}, {}, false});
  distant_map.parking_spots.push_back({"P", {{10.0, 0.0}, 0.0}, {{10.0, 0.0}, 0.0}});
  Check(!planner.Plan(MakeGlobalFrame(&distant_map, {{0.0, 2.1}, 0.0}, "P"), &route, &error),
        "ego farther than lane matching threshold must be rejected");
}

void TestMapValidation() {
  avp::Planner planner;
  avp::PlanningRequest duplicate_lane = MakeRequest();
  duplicate_lane.map.lanes.push_back(duplicate_lane.map.lanes.front());
  Check(planner.Plan(duplicate_lane).status == avp::PlanningStatus::kInvalidInput,
        "duplicate lane ids must be rejected");

  avp::PlanningRequest unknown_successor = MakeRequest();
  unknown_successor.map.lanes.front().successor_ids = {"missing"};
  Check(planner.Plan(unknown_successor).status == avp::PlanningStatus::kInvalidInput,
        "unknown lane successor must be rejected");

  avp::PlanningRequest invalid_centerline = MakeRequest();
  invalid_centerline.map.lanes.front().centerline[1].x =
      std::numeric_limits<double>::quiet_NaN();
  Check(planner.Plan(invalid_centerline).status == avp::PlanningStatus::kInvalidInput,
        "non-finite lane centerline point must be rejected");

  avp::PlanningRequest duplicate_point = MakeRequest();
  duplicate_point.map.lanes.front().centerline[1] = duplicate_point.map.lanes.front().centerline[0];
  Check(planner.Plan(duplicate_point).status == avp::PlanningStatus::kInvalidInput,
        "zero-length lane segment must be rejected");

  avp::PlanningRequest duplicate_parking_spot = MakeRequest();
  duplicate_parking_spot.map.parking_spots.push_back(duplicate_parking_spot.map.parking_spots.front());
  Check(planner.Plan(duplicate_parking_spot).status == avp::PlanningStatus::kInvalidInput,
        "duplicate parking spot ids must be rejected");

  avp::PlanningRequest invalid_parking_pose = MakeRequest();
  invalid_parking_pose.map.parking_spots.front().entry_pose.yaw =
      std::numeric_limits<double>::infinity();
  Check(planner.Plan(invalid_parking_pose).status == avp::PlanningStatus::kInvalidInput,
        "non-finite parking pose must be rejected");

  avp::PlannerConfig invalid_match_config;
  invalid_match_config.max_lane_match_distance_m = 0.0;
  avp::Planner invalid_match_planner({}, invalid_match_config);
  Check(invalid_match_planner.Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "non-positive lane match distance must be rejected");

  invalid_match_config.max_lane_match_distance_m = 2.0;
  invalid_match_config.max_lane_heading_difference_rad = avp::kPi + 1e-3;
  avp::Planner invalid_heading_planner({}, invalid_match_config);
  Check(invalid_heading_planner.Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "lane heading threshold above pi must be rejected");
}

void TestPlannerTrajectoryStartsAtEgoPose() {
  avp::PlanningRequest request = MakeRequest();
  request.ego.pose = {{2.5, 0.0}, 0.0};
  avp::Planner planner;
  const avp::PlanningResponse response = planner.Plan(request);
  Check(response.status == avp::PlanningStatus::kOk,
        "planner should generate a trajectory from a mid-lane ego pose");
  Check(!response.trajectory.empty(), "trajectory should contain the ego anchor point");
  CheckPosition(response.trajectory.front().pose.position, request.ego.pose.position,
                "timed trajectory must start at ego position");
  Check(NearlyEqual(response.trajectory.front().pose.yaw, request.ego.pose.yaw),
        "timed trajectory must start at ego yaw");
}

void TestLocalPlannerAnchorsEgoPose() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.ego.pose = {{0.0, 0.0}, 0.0};
  frame.vehicle.length_m = 1.0;
  frame.vehicle.width_m = 0.4;
  frame.vehicle.max_curvature_1pm = 1.0;
  frame.vehicle.safety_margin_m = 0.05;
  frame.config.path_step_m = 0.5;
  avp::GlobalRoute route;
  route.reference_line = {{0.0, 0.0}, {10.0, 0.0}};
  avp::LocalPlanner planner;
  std::vector<avp::PathPoint> path;
  std::string error;
  Check(planner.Plan(frame, route, {}, &path, &error), "local planner should plan from ego pose");
  CheckPosition(path.front().position, frame.ego.pose.position,
                "local path first point must equal ego position");
  Check(NearlyEqual(path.front().yaw, frame.ego.pose.yaw),
        "local path first yaw must equal ego yaw");

  frame.ego.pose.yaw = avp::kPi;
  frame.vehicle.max_curvature_1pm = 0.1;
  Check(!planner.Plan(frame, route, {}, &path, &error),
        "infeasible ego anchor heading must reject the local path");
}

void TestOrientedRectangleCollision() {
  for (const avp::test::CollisionTestCase& test_case : avp::test::CollisionTestCases()) {
    const bool collision = avp::IsVehicleObstacleCollision(
        test_case.vehicle_pose, test_case.vehicle, test_case.obstacle_pose,
        test_case.obstacle_length_m, test_case.obstacle_width_m);
    Check(collision == test_case.expected_collision, test_case.failure_message);
  }
}

void TestSlLattice() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.vehicle.length_m = 1.0;
  frame.vehicle.width_m = 0.4;
  frame.vehicle.max_curvature_1pm = 1.0;
  frame.vehicle.safety_margin_m = 0.05;
  frame.config.path_step_m = 0.5;
  frame.obstacles.push_back({"barrel", 0.2, 0.2, 1.0,
                             {{{1'000'000'000, {{10.0, 0.0}, 0.0}, 0.0}}}});
  avp::GlobalRoute route;
  route.reference_line = {{0.0, 0.0}, {20.0, 0.0}};
  avp::LocalPlanner planner;
  std::vector<avp::PathPoint> path;
  std::string error;
  Check(planner.Plan(frame, route, {}, &path, &error), "S-L lattice should find a detour");
  Check(path.size() > route.reference_line.size(), "S-L lattice should resample reference layers");
  bool detours_around_obstacle = false;
  for (const avp::PathPoint& point : path) {
    if (point.s > 8.0 && point.s < 12.0 && std::abs(point.position.y) > 0.3) {
      detours_around_obstacle = true;
    }
  }
  Check(detours_around_obstacle, "S-L lattice should choose a lateral detour");
}

void TestAccelerationConstrainedStDp() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.config.horizon_s = 1.5;
  frame.config.time_step_s = 0.5;
  frame.config.jerk_weight = 0.0;
  frame.vehicle.max_speed_mps = 2.0;
  frame.vehicle.max_acceleration_mps2 = 1.0;
  frame.vehicle.max_deceleration_mps2 = 1.0;
  frame.vehicle.min_jerk_mps3 = -1.0;
  frame.vehicle.max_jerk_mps3 = 1.0;
  const std::vector<avp::PathPoint> path{{{0.0, 0.0}, 0.0, 0.0, 0.0},
                                          {{0.125, 0.0}, 0.0, 0.0, 0.125},
                                          {{0.5, 0.0}, 0.0, 0.0, 0.5},
                                          {{1.0, 0.0}, 0.0, 0.0, 1.0}};
  avp::SpeedPlanner planner;
  std::vector<avp::SpeedPoint> profile;
  std::string error;
  Check(planner.Plan(frame, path, &profile, &error),
        "acceleration- and jerk-constrained S-T DP should plan");
  Check(profile.size() == 4, "S-T profile should cover every time layer");
  Check(profile.back().s > 0.0, "jerk-feasible S-T profile should make progress");
  for (size_t index = 1; index < profile.size(); ++index) {
    const double acceleration =
        (profile[index].speed_mps - profile[index - 1].speed_mps) / frame.config.time_step_s;
    Check(acceleration <= frame.vehicle.max_acceleration_mps2 + 1e-9,
          "S-T transition must respect maximum acceleration");
    Check(acceleration >= -frame.vehicle.max_deceleration_mps2 - 1e-9,
          "S-T transition must respect maximum deceleration");
    const double jerk =
        (profile[index].acceleration_mps2 - profile[index - 1].acceleration_mps2) /
        frame.config.time_step_s;
    Check(jerk <= frame.vehicle.max_jerk_mps3 + 1e-9,
          "S-T transition must respect maximum positive jerk");
    Check(jerk >= frame.vehicle.min_jerk_mps3 - 1e-9,
          "S-T transition must respect maximum negative jerk");
  }
  const double first_jerk =
      (profile[1].acceleration_mps2 - profile[0].acceleration_mps2) /
      frame.config.time_step_s;
  const double last_jerk =
      (profile[3].acceleration_mps2 - profile[2].acceleration_mps2) /
      frame.config.time_step_s;
  Check(NearlyEqual(first_jerk, frame.vehicle.max_jerk_mps3),
        "test profile should exercise the positive jerk boundary");
  Check(NearlyEqual(last_jerk, frame.vehicle.min_jerk_mps3),
        "test profile should exercise the negative jerk boundary");
}

void TestJerkUsesInitialAccelerationAndShortHorizons() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.config.horizon_s = 0.5;
  frame.config.time_step_s = 0.5;
  frame.config.jerk_weight = 0.0;
  frame.ego.speed_mps = 0.5;
  frame.ego.acceleration_mps2 = 0.5;
  frame.vehicle.max_speed_mps = 2.0;
  frame.vehicle.max_acceleration_mps2 = 1.0;
  frame.vehicle.max_deceleration_mps2 = 1.0;
  frame.vehicle.min_jerk_mps3 = -0.1;
  frame.vehicle.max_jerk_mps3 = 0.1;
  const std::vector<avp::PathPoint> two_layer_path{
      {{0.0, 0.0}, 0.0, 0.0, 0.0}, {{0.375, 0.0}, 0.0, 0.0, 0.375}};
  avp::SpeedPlanner planner;
  std::vector<avp::SpeedPoint> profile;
  std::string error;
  Check(planner.Plan(frame, two_layer_path, &profile, &error),
        "two-layer S-T horizon should use the ego acceleration for the first jerk");
  Check(profile.size() == 2 && NearlyEqual(profile.back().s, 0.375),
        "two-layer S-T horizon should choose the only initial-jerk-feasible transition");
  Check(NearlyEqual(profile[1].acceleration_mps2, frame.ego.acceleration_mps2),
        "first planned acceleration should be continuous with ego acceleration");

  frame.config.horizon_s = 1.0;
  frame.ego.speed_mps = 0.0;
  frame.ego.acceleration_mps2 = 0.0;
  frame.vehicle.min_jerk_mps3 = -1.0;
  frame.vehicle.max_jerk_mps3 = 1.0;
  const std::vector<avp::PathPoint> three_layer_path{{{0.0, 0.0}, 0.0, 0.0, 0.0},
                                                      {{0.125, 0.0}, 0.0, 0.0, 0.125},
                                                      {{0.5, 0.0}, 0.0, 0.0, 0.5}};
  Check(planner.Plan(frame, three_layer_path, &profile, &error),
        "three-layer S-T horizon should initialize a complete jerk state");
  Check(profile.size() == 3 && NearlyEqual(profile.back().s, 0.5),
        "three-layer S-T horizon should backtrack its terminal state");
}

void TestJerkCostPrefersSmootherProfile() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.config.horizon_s = 1.5;
  frame.config.time_step_s = 0.5;
  frame.vehicle.max_speed_mps = 2.0;
  frame.vehicle.max_acceleration_mps2 = 4.0;
  frame.vehicle.max_deceleration_mps2 = 4.0;
  frame.vehicle.min_jerk_mps3 = -20.0;
  frame.vehicle.max_jerk_mps3 = 20.0;
  const std::vector<avp::PathPoint> path{{{0.0, 0.0}, 0.0, 0.0, 0.0},
                                          {{0.125, 0.0}, 0.0, 0.0, 0.125},
                                          {{0.5, 0.0}, 0.0, 0.0, 0.5},
                                          {{1.0, 0.0}, 0.0, 0.0, 1.0}};
  avp::SpeedPlanner planner;
  std::vector<avp::SpeedPoint> unweighted_profile;
  std::vector<avp::SpeedPoint> weighted_profile;
  std::string error;
  frame.config.jerk_weight = 0.0;
  Check(planner.Plan(frame, path, &unweighted_profile, &error),
        "unweighted jerk profile should plan");
  frame.config.jerk_weight = 1.0;
  Check(planner.Plan(frame, path, &weighted_profile, &error),
        "weighted jerk profile should plan");
  Check(SumSquaredJerk(weighted_profile, frame.config.time_step_s) <
            SumSquaredJerk(unweighted_profile, frame.config.time_step_s),
        "jerk cost should select a strictly smoother profile on the crafted lattice");
}

void TestNoJerkFeasibleSpeedProfile() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  frame.config.horizon_s = 0.5;
  frame.config.time_step_s = 0.5;
  frame.ego.acceleration_mps2 = 1.0;
  frame.vehicle.max_acceleration_mps2 = 1.0;
  frame.vehicle.min_jerk_mps3 = -0.5;
  frame.vehicle.max_jerk_mps3 = 0.5;
  const std::vector<avp::PathPoint> path{{{0.0, 0.0}, 0.0, 0.0, 0.0}};
  avp::SpeedPlanner planner;
  std::vector<avp::SpeedPoint> profile;
  std::string error;
  Check(!planner.Plan(frame, path, &profile, &error),
        "S-T DP should reject a lattice with no jerk-feasible first transition");
  Check(error == "all acceleration- and jerk-feasible S-T lattice states are blocked",
        "jerk infeasibility should be reported explicitly");
}

void TestConfigurationValidation() {
  avp::VehicleConfig vehicle;
  vehicle.min_jerk_mps3 = 0.0;
  Check(avp::Planner(vehicle).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "non-negative minimum jerk must be rejected");
  vehicle = {};
  vehicle.max_jerk_mps3 = 0.0;
  Check(avp::Planner(vehicle).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "non-positive maximum jerk must be rejected");
  vehicle = {};
  vehicle.min_jerk_mps3 = std::numeric_limits<double>::quiet_NaN();
  Check(avp::Planner(vehicle).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "non-finite minimum jerk must be rejected");
  for (const double invalid_wheelbase :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
    vehicle = {};
    vehicle.wheelbase_m = invalid_wheelbase;
    Check(avp::Planner(vehicle).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
          "non-positive and non-finite wheelbase must be rejected");
  }

  avp::PlannerConfig config;
  config.jerk_weight = -1.0;
  Check(avp::Planner({}, config).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "negative jerk weight must be rejected");
  config = {};
  config.jerk_weight = std::numeric_limits<double>::infinity();
  Check(avp::Planner({}, config).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "non-finite jerk weight must be rejected");
  config = {};
  config.max_reverse_speed_mps = 4.0;
  Check(avp::Planner({}, config).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "reverse speed above the vehicle limit must be rejected");
  config = {};
  config.gear_shift_dwell_s = -0.1;
  Check(avp::Planner({}, config).Plan(MakeRequest()).status == avp::PlanningStatus::kInvalidInput,
        "negative gear-shift dwell must be rejected");
}

void TestPlanningFrameAdapterFailureIsAtomic() {
  avp::MapSnapshot sentinel_map;
  avp::PlanningFrame frame;
  frame.header = {"sentinel", 42, 7};
  frame.ego.pose = {{1.0, 2.0}, 0.3};
  frame.ego.speed_mps = 0.4;
  frame.map = &sentinel_map;
  frame.target_parking_spot_id = "sentinel_target";
  frame.obstacles.push_back({});
  frame.obstacles.front().id = "sentinel_obstacle";
  frame.vehicle.length_m = 9.0;
  frame.vehicle.wheelbase_m = 5.0;
  frame.config.horizon_s = 99.0;
  frame.config.time_step_s = 0.1;

  avp::PlanningRequest invalid_request = MakeRequest();
  invalid_request.target_parking_spot_id.clear();
  avp::PlanningFrameAdapter adapter;
  std::string error;
  Check(!adapter.Adapt(invalid_request, {}, {}, &frame, &error),
        "invalid task must fail frame adaptation");
  Check(error == "target parking spot is required", "task validation error must be preserved");
  Check(frame.header.frame_id == "sentinel" && frame.header.timestamp_ns == 42 &&
            frame.header.sequence_id == 7,
        "failed adaptation must preserve the output header");
  CheckPosition(frame.ego.pose.position, {1.0, 2.0},
                "failed adaptation must preserve the output ego pose");
  Check(NearlyEqual(frame.ego.pose.yaw, 0.3) && NearlyEqual(frame.ego.speed_mps, 0.4),
        "failed adaptation must preserve the output ego state");
  Check(frame.map == &sentinel_map, "failed adaptation must preserve the output map pointer");
  Check(frame.target_parking_spot_id == "sentinel_target",
        "failed adaptation must preserve the output task");
  Check(frame.obstacles.size() == 1 && frame.obstacles.front().id == "sentinel_obstacle",
        "failed adaptation must preserve the output obstacles");
  Check(NearlyEqual(frame.vehicle.length_m, 9.0) && NearlyEqual(frame.vehicle.wheelbase_m, 5.0),
        "failed adaptation must preserve the output vehicle configuration");
  Check(NearlyEqual(frame.config.horizon_s, 99.0) && NearlyEqual(frame.config.time_step_s, 0.1),
        "failed adaptation must preserve the output planner configuration");
}

void TestObstacleDimensionValidation() {
  avp::Planner planner;
  for (const double invalid_dimension :
       {0.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
    avp::PlanningRequest request = MakeRequest();
    request.obstacles.push_back(
        {"invalid", invalid_dimension, 1.0, 1.0,
         {{{1'000'000'000, {{5.0, 0.0}, 0.0}, 0.0}}}});
    Check(planner.Plan(request).status == avp::PlanningStatus::kInvalidInput,
          "invalid obstacle length must be rejected");
  }
  avp::PlanningRequest invalid_width = MakeRequest();
  invalid_width.obstacles.push_back(
      {"invalid", 1.0, std::numeric_limits<double>::infinity(), 1.0,
       {{{1'000'000'000, {{5.0, 0.0}, 0.0}, 0.0}}}});
  Check(planner.Plan(invalid_width).status == avp::PlanningStatus::kInvalidInput,
        "invalid obstacle width must be rejected");
}

void TestRollingLocalHorizonIgnoresFarObstacle() {
  avp::Planner planner;
  for (const double route_length : {100.0, 1000.0}) {
    avp::PlanningRequest request;
    request.header = {"map", 1'000'000'000 + static_cast<uint64_t>(route_length),
                      static_cast<uint64_t>(route_length)};
    request.ego.pose = {{0.0, 0.0}, 0.0};
    request.map.lanes.push_back(
        {"long", {{0.0, 0.0}, {route_length, 0.0}}, {}, false});
    request.map.parking_spots.push_back(
        {"P", {{route_length, 0.0}, 0.0}, {{route_length + 1.0, 0.0}, 0.0}});
    request.target_parking_spot_id = "P";
    request.obstacles.push_back(
        {"far", 1.0, 1.0, 1.0,
         {{request.header.timestamp_ns, {{0.5 * route_length, 0.0}, 0.0}, 0.0}}});

    const avp::PlanningResponse response = planner.Plan(request);
    Check(response.status == avp::PlanningStatus::kOk,
          "obstacle beyond the rolling horizon must not block the current trajectory");
    const double local_horizon = DiagnosticValue(response, "local_horizon_m=");
    const double layer_count = DiagnosticValue(response, "path_layer_count=");
    Check(std::isfinite(local_horizon) && local_horizon >= 25.0 && local_horizon <= 30.0,
          "default rolling horizon should be derived from reachability and stopping distance");
    Check(std::isfinite(layer_count) && layer_count <= 65.0,
          "S-L layer count must stay bounded on 100-meter and one-kilometer routes");
    Check(response.trajectory.size() == 41,
          "rolling spatial horizon must not change the default temporal horizon");
  }
}

void TestHybridAStarPreservesReverseDirection() {
  avp::PlanningFrame frame;
  frame.header = {"map", 1'000'000'000, 1};
  avp::HybridAStar planner;
  avp::ParkingManeuver maneuver;
  std::string error;
  Check(planner.Plan(frame, {{0.0, 0.0}, 0.0}, {{-1.0, 0.0}, 0.0}, &maneuver, &error),
        "Hybrid A* should find a straight reverse parking maneuver");
  Check(!maneuver.segments.empty(), "reverse maneuver must contain a direction segment");
  Check(maneuver.segments.front().direction == avp::DrivingDirection::kReverse,
        "a target behind the vehicle must retain REVERSE direction");
  for (const avp::HybridPathPoint& point : maneuver.segments.front().points) {
    Check(std::abs(avp::NormalizeAngle(point.pose.yaw)) < 1e-6,
          "reverse path must preserve body yaw instead of travel tangent yaw");
  }
}

void CheckCuspManeuver(const avp::Pose2d& goal, avp::DrivingDirection first,
                       avp::DrivingDirection second, const char* message) {
  avp::PlanningFrame frame;
  avp::HybridAStar planner;
  avp::ParkingManeuver maneuver;
  std::string error;
  Check(planner.Plan(frame, {{0.0, 0.0}, 0.0}, goal, &maneuver, &error), message);
  Check(maneuver.segments.size() >= 2 && maneuver.segments[0].direction == first &&
            maneuver.segments[1].direction == second,
        message);
  CheckPosition(maneuver.segments[0].points.back().pose.position,
                maneuver.segments[1].points.front().pose.position,
                "adjacent gear segments must share the same cusp position");
  for (const avp::ParkingSegment& segment : maneuver.segments) {
    Check(segment.points.size() >= 2, "every gear partition must contain usable motion");
    for (const avp::HybridPathPoint& point : segment.points) {
      Check(point.direction == segment.direction,
            "a parking partition must not mix motion directions");
    }
  }
}

void TestHybridAStarPartitionsCusps() {
  CheckCuspManeuver({{-3.0, -3.0}, 0.0}, avp::DrivingDirection::kReverse,
                    avp::DrivingDirection::kDrive,
                    "Hybrid A* should partition a reverse-to-drive cusp");
  CheckCuspManeuver({{3.0, -2.0}, 0.0}, avp::DrivingDirection::kDrive,
                    avp::DrivingDirection::kReverse,
                    "Hybrid A* should partition a drive-to-reverse cusp");
}

void TestParkingGearShiftAndReverseFallback() {
  avp::PlanningRequest request;
  request.header = {"map", 1'000'000'000, 1};
  request.ego.pose = {{10.0, 0.0}, 0.0};
  request.ego.direction = avp::DrivingDirection::kDrive;
  request.map.lanes.push_back({"entry", {{0.0, 0.0}, {10.0, 0.0}}, {}, false});
  request.map.parking_spots.push_back(
      {"P", {{10.0, 0.0}, 0.0}, {{9.0, 0.0}, 0.0}});
  request.target_parking_spot_id = "P";

  avp::Planner planner;
  const avp::PlanningResponse shift = planner.Plan(request);
  Check(shift.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(shift, "planning_mode=GEAR_SHIFT"),
        "reverse parking must first publish a stopped gear-shift trajectory");
  for (const avp::TimedTrajectoryPoint& point : shift.trajectory) {
    Check(point.direction == avp::DrivingDirection::kReverse &&
              NearlyEqual(point.speed_mps, 0.0),
          "gear-shift trajectory must request REVERSE while remaining stopped");
  }

  request.header.timestamp_ns += 1'100'000'000;
  request.header.sequence_id += 1;
  request.ego.direction = avp::DrivingDirection::kReverse;
  const avp::PlanningResponse reverse = planner.Plan(request);
  Check(reverse.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(reverse, "planning_mode=OPEN_SPACE_PARKING"),
        "parking should enter the reverse segment after dwell and gear feedback");
  for (const avp::TimedTrajectoryPoint& point : reverse.trajectory) {
    Check(point.direction == avp::DrivingDirection::kReverse,
          "a parking trajectory must never mix drive and reverse points");
    Check(point.speed_mps <= 1.0 + 1e-9, "reverse speed limit must be enforced");
  }
  Check(NearlyEqual(reverse.trajectory.back().speed_mps, 0.0),
        "a reachable parking segment must end at zero speed");

  request.header.timestamp_ns += 200'000'000;
  request.header.sequence_id += 1;
  request.obstacles.push_back(
      {"blocking", 2.0, 2.0, 1.0,
       {{request.header.timestamp_ns, request.ego.pose, 0.0}}});
  const avp::PlanningResponse fallback = planner.Plan(request);
  Check(fallback.status == avp::PlanningStatus::kNoSafeTrajectory,
        "blocked reverse parking must use the emergency fallback");
  Check(!fallback.trajectory.empty() &&
            fallback.trajectory.front().direction == avp::DrivingDirection::kReverse,
        "reverse emergency fallback must preserve the active gear");
  for (const avp::TimedTrajectoryPoint& point : fallback.trajectory) {
    Check(point.pose.position.x <= request.ego.pose.position.x + 1e-9,
          "reverse emergency fallback must move opposite the body heading");
  }
}

void TestParkingDeviationReplanIsBoundedPerFrame() {
  avp::PlanningRequest request;
  request.header = {"map", 1'000'000'000, 1};
  request.ego.pose = {{10.0, 0.0}, 0.0};
  request.ego.direction = avp::DrivingDirection::kDrive;
  request.map.lanes.push_back({"entry", {{0.0, 0.0}, {10.0, 0.0}}, {}, false});
  request.map.parking_spots.push_back(
      {"P", {{10.0, 0.0}, 0.0}, {{9.0, 0.0}, 0.0}});
  request.target_parking_spot_id = "P";

  avp::Planner planner;
  const avp::PlanningResponse shift = planner.Plan(request);
  Check(shift.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(shift, "planning_mode=GEAR_SHIFT"),
        "deviation test must first enter the reverse gear-shift state");

  request.header.timestamp_ns += 1'100'000'000;
  request.header.sequence_id += 1;
  request.ego.direction = avp::DrivingDirection::kReverse;
  const avp::PlanningResponse initial_parking = planner.Plan(request);
  Check(initial_parking.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(initial_parking, "planning_mode=OPEN_SPACE_PARKING"),
        "deviation test must first activate the original parking maneuver");

  request.header.timestamp_ns += 200'000'000;
  request.header.sequence_id += 1;
  request.ego.pose.position.x = 12.0;
  const avp::PlanningResponse replanned = planner.Plan(request);
  Check(replanned.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(replanned, "planning_mode=OPEN_SPACE_PARKING"),
        "the first parking-path deviation must replan from the current pose");
  Check(!replanned.trajectory.empty(), "a successful deviation replan must publish a trajectory");
  CheckPosition(replanned.trajectory.front().pose.position, request.ego.pose.position,
                "a deviation replan must anchor its trajectory at the current ego pose");

  // 保持同一时间戳，再次移动到新轨迹之外，验证不会在一个规划帧内重复重规划。
  request.ego.pose.position.x = 14.0;
  const avp::PlanningResponse limited = planner.Plan(request);
  Check(limited.status == avp::PlanningStatus::kNoSafeTrajectory,
        "a repeated deviation in the same frame must use the fallback");
  Check(limited.message == "parking replan already attempted for current frame",
        "the repeated deviation must report the per-frame replan limit");
}

void TestParkingCuspRequiresStopAndDwell() {
  avp::PlanningFrame hybrid_frame;
  avp::HybridAStar hybrid;
  avp::ParkingManeuver maneuver;
  std::string error;
  const avp::Pose2d goal{{-3.0, -3.0}, 0.0};
  Check(hybrid.Plan(hybrid_frame, {{0.0, 0.0}, 0.0}, goal, &maneuver, &error) &&
            maneuver.segments.size() >= 2,
        "cusp state-machine test requires a multi-gear maneuver");
  const avp::Pose2d cusp = maneuver.segments[0].points.back().pose;

  avp::PlanningRequest request;
  request.header = {"map", 1'000'000'000, 1};
  request.ego.pose = {{0.0, 0.0}, 0.0};
  request.ego.direction = avp::DrivingDirection::kReverse;
  request.map.lanes.push_back({"entry", {{-5.0, 0.0}, {0.0, 0.0}}, {}, false});
  request.map.parking_spots.push_back({"P", {{0.0, 0.0}, 0.0}, goal});
  request.target_parking_spot_id = "P";

  avp::Planner planner;
  const avp::PlanningResponse first_segment = planner.Plan(request);
  Check(first_segment.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(first_segment, "gear=REVERSE"),
        "parking should publish only the first reverse segment before the cusp");

  request.header.timestamp_ns += 200'000'000;
  request.header.sequence_id += 1;
  request.ego.pose = cusp;
  request.ego.speed_mps = 0.0;
  const avp::PlanningResponse shift = planner.Plan(request);
  Check(shift.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(shift, "planning_mode=GEAR_SHIFT") && HasDiagnostic(shift, "gear=DRIVE"),
        "reaching a cusp at zero speed must start the next gear-shift dwell");
  for (const avp::TimedTrajectoryPoint& point : shift.trajectory) {
    Check(NearlyEqual(point.speed_mps, 0.0), "cusp gear-shift dwell must remain stationary");
  }

  request.header.timestamp_ns += 1'100'000'000;
  request.header.sequence_id += 1;
  request.ego.direction = avp::DrivingDirection::kDrive;
  const avp::PlanningResponse next_segment = planner.Plan(request);
  Check(next_segment.status == avp::PlanningStatus::kOk &&
            HasDiagnostic(next_segment, "planning_mode=OPEN_SPACE_PARKING") &&
            HasDiagnostic(next_segment, "gear=DRIVE"),
        "the next drive segment may start only after dwell and gear feedback");
  for (const avp::TimedTrajectoryPoint& point : next_segment.trajectory) {
    Check(point.direction == avp::DrivingDirection::kDrive,
          "post-cusp output must contain only the new gear");
  }
}
}  // 匿名命名空间

int main() {
  TestOrientedRectangleCollision();
  TestGlobalRouteUsesLaneProjections();
  TestSameLaneEntryBehindEgoIsUnreachable();
  TestLaneProjectionMatching();
  TestMapValidation();
  TestPlannerTrajectoryStartsAtEgoPose();
  TestLocalPlannerAnchorsEgoPose();
  TestSlLattice();
  TestAccelerationConstrainedStDp();
  TestJerkUsesInitialAccelerationAndShortHorizons();
  TestJerkCostPrefersSmootherProfile();
  TestNoJerkFeasibleSpeedProfile();
  TestConfigurationValidation();
  TestPlanningFrameAdapterFailureIsAtomic();
  TestObstacleDimensionValidation();
  TestRollingLocalHorizonIgnoresFarObstacle();
  TestHybridAStarPreservesReverseDirection();
  TestHybridAStarPartitionsCusps();
  TestParkingGearShiftAndReverseFallback();
  TestParkingDeviationReplanIsBoundedPerFrame();
  TestParkingCuspRequiresStopAndDwell();
  avp::Planner planner;
  const avp::PlanningRequest request = MakeRequest();
  const avp::PlanningResponse first = planner.Plan(request);
  const avp::PlanningResponse second = planner.Plan(request);
  Check(first.status == avp::PlanningStatus::kOk, "nominal route should be planned");
  Check(!first.trajectory.empty(), "trajectory should not be empty");
  Check(first.trajectory.size() == second.trajectory.size(), "planner must be deterministic");
  avp::PlanningRequest invalid = request;
  invalid.header.frame_id = "odom";
  Check(planner.Plan(invalid).status == avp::PlanningStatus::kInvalidInput, "frame must be map");
  avp::PlanningRequest unknown_spot = request;
  unknown_spot.target_parking_spot_id = "unknown";
  Check(planner.Plan(unknown_spot).status == avp::PlanningStatus::kNoRoute,
        "unknown parking spot should have no route");
  avp::PlanningRequest blocked = request;
  blocked.obstacles.push_back(
      {"wall", 1.0, 6.0, 1.0, {{{1'000'000'000, {{5.0, 0.0}, 0.0}, 0.0}}}});
  const avp::PlanningResponse blocked_response = planner.Plan(blocked);
  Check(blocked_response.status == avp::PlanningStatus::kNoSafeTrajectory,
        "blocked route should fall back");
  Check(HasDiagnostic(blocked_response, "jerk_constraint=emergency_exempt"),
        "emergency fallback should diagnose its jerk-constraint exemption");
  std::cout << "avp_planning_test passed\n";
}
