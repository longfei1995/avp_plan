#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>

#include "local/local_planner.h"
#include "planning/planner.h"
#include "speed/speed_planner.h"

namespace {
void Check(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
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

void TestOrientedRectangleCollision() {
  avp::VehicleConfig vehicle;
  vehicle.length_m = 4.0;
  vehicle.width_m = 2.0;
  vehicle.safety_margin_m = 0.0;
  const avp::Pose2d pose{{0.0, 0.0}, 0.0};

  Check(avp::IsVehicleObstacleCollision(pose, vehicle, {{2.5, 0.0}, 0.0}, 2.0, 1.0),
        "axis-aligned rectangles must collide when they overlap");
  Check(!avp::IsVehicleObstacleCollision(pose, vehicle, {{3.5, 0.0}, 0.0}, 2.0, 1.0),
        "axis-aligned rectangles must not collide when separated");
  Check(avp::IsVehicleObstacleCollision(pose, vehicle, {{2.5, 0.0}, avp::kPi / 4.0}, 4.0,
                                         2.0),
        "rotated rectangles must collide when their OBBs overlap");
  Check(!avp::IsVehicleObstacleCollision(pose, vehicle, {{3.7, 2.3}, avp::kPi / 4.0}, 4.0,
                                          2.0),
        "SAT must separate rotated rectangles despite overlapping AABBs");
  Check(avp::IsVehicleObstacleCollision(pose, vehicle, {{3.0, 0.0}, 0.0}, 2.0, 1.0),
        "rectangle boundary contact must count as a collision");

  vehicle.safety_margin_m = 0.6;
  Check(avp::IsVehicleObstacleCollision(pose, vehicle, {{3.5, 0.0}, 0.0}, 2.0, 1.0),
        "safety margin must inflate the vehicle collision envelope");
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
  frame.vehicle.max_speed_mps = 2.0;
  frame.vehicle.max_acceleration_mps2 = 0.5;
  frame.vehicle.max_deceleration_mps2 = 0.5;
  const std::vector<avp::PathPoint> path{{{0.0, 0.0}, 0.0, 0.0, 0.0},
                                          {{0.125, 0.0}, 0.0, 0.0, 0.125},
                                          {{0.5, 0.0}, 0.0, 0.0, 0.5},
                                          {{1.125, 0.0}, 0.0, 0.0, 1.125}};
  avp::SpeedPlanner planner;
  std::vector<avp::SpeedPoint> profile;
  std::string error;
  Check(planner.Plan(frame, path, &profile, &error), "acceleration-constrained S-T DP should plan");
  Check(profile.size() == 4, "S-T profile should cover every time layer");
  for (size_t index = 1; index < profile.size(); ++index) {
    const double acceleration =
        (profile[index].speed_mps - profile[index - 1].speed_mps) / frame.config.time_step_s;
    Check(acceleration <= frame.vehicle.max_acceleration_mps2 + 1e-9,
          "S-T transition must respect maximum acceleration");
    Check(acceleration >= -frame.vehicle.max_deceleration_mps2 - 1e-9,
          "S-T transition must respect maximum deceleration");
  }
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
}  // 匿名命名空间

int main() {
  TestOrientedRectangleCollision();
  TestSlLattice();
  TestAccelerationConstrainedStDp();
  TestObstacleDimensionValidation();
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
  Check(planner.Plan(blocked).status == avp::PlanningStatus::kNoSafeTrajectory, "blocked route should fall back");
  std::cout << "avp_planning_test passed\n";
}
