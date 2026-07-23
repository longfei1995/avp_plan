#ifndef AVP_TESTS_COLLISION_TEST_CASES_H_
#define AVP_TESTS_COLLISION_TEST_CASES_H_

#include <array>

#include "common/types.h"

namespace avp::test {

struct CollisionTestCase {
  const char* name;
  const char* failure_message;
  VehicleConfig vehicle;
  Pose2d vehicle_pose;
  Pose2d obstacle_pose;
  double obstacle_length_m = 0.0;
  double obstacle_width_m = 0.0;
  bool expected_collision = false;
};

inline const std::array<CollisionTestCase, 6>& CollisionTestCases() {
  static const std::array<CollisionTestCase, 6> kCases = [] {
    VehicleConfig vehicle;
    vehicle.length_m = 4.0;
    vehicle.width_m = 2.0;
    vehicle.safety_margin_m = 0.0;
    const Pose2d vehicle_pose{{0.0, 0.0}, 0.0};

    VehicleConfig vehicle_with_margin = vehicle;
    vehicle_with_margin.safety_margin_m = 0.6;

    return std::array<CollisionTestCase, 6>{
        {{"axis_aligned_overlap", "axis-aligned rectangles must collide when they overlap",
          vehicle, vehicle_pose, {{2.5, 0.0}, 0.0}, 2.0, 1.0, true},
         {"axis_aligned_separation", "axis-aligned rectangles must not collide when separated",
          vehicle, vehicle_pose, {{3.5, 0.0}, 0.0}, 2.0, 1.0, false},
         {"rotated_overlap", "rotated rectangles must collide when their OBBs overlap", vehicle,
          vehicle_pose, {{2.5, 0.0}, kPi / 4.0}, 4.0, 2.0, true},
         {"rotated_sat_separation",
          "SAT must separate rotated rectangles despite overlapping AABBs", vehicle, vehicle_pose,
          {{3.7, 2.3}, kPi / 4.0}, 4.0, 2.0, false},
         {"boundary_contact", "rectangle boundary contact must count as a collision", vehicle,
          vehicle_pose, {{3.0, 0.0}, 0.0}, 2.0, 1.0, true},
         {"safety_margin", "safety margin must inflate the vehicle collision envelope",
          vehicle_with_margin, vehicle_pose, {{3.5, 0.0}, 0.0}, 2.0, 1.0, true}}};
  }();
  return kCases;
}

}  // namespace avp::test

#endif  // AVP_TESTS_COLLISION_TEST_CASES_H_
