#include "interfaces/adapters.h"

#include <cmath>
#include <unordered_set>

namespace avp {
namespace {
// 数值是否有效（不是无穷大，不是nan)
bool IsFinite(double value) { return std::isfinite(value); }
bool IsValidPose(const Pose2d& pose) {
  return IsFinite(pose.position.x) && IsFinite(pose.position.y) && IsFinite(pose.yaw);
}
}  // namespace

bool LocalizationAdapter::Adapt(const PlanningRequest& request, PlanningFrame* frame,
                                std::string* error) const {
  if (request.header.frame_id != "map") {
    *error = "planning input must be normalized to the map frame";
    return false;
  }
  if (request.header.timestamp_ns == 0 || request.header.sequence_id == 0) {
    *error = "header timestamp and sequence_id are required";
    return false;
  }
  if (!IsValidPose(request.ego.pose) || !IsFinite(request.ego.speed_mps) ||
      !IsFinite(request.ego.acceleration_mps2)) {
    *error = "invalid ego state";
    return false;
  }
  frame->header = request.header;
  frame->ego = request.ego;
  return true;
}

bool PerceptionAdapter::Adapt(const PlanningRequest& request, PlanningFrame* frame,
                              std::string* error) const {
  for (const Obstacle& obstacle : request.obstacles) {
    if (obstacle.id.empty() || !IsFinite(obstacle.length_m) || !IsFinite(obstacle.width_m) ||
        obstacle.length_m <= 0.0 || obstacle.width_m <= 0.0 || obstacle.confidence < 0.0 ||
        obstacle.confidence > 1.0 || obstacle.prediction.empty()) {
      *error = "obstacle id, positive dimensions, confidence, and prediction are required";
      return false;
    }
    for (const PredictionPoint& point : obstacle.prediction) {
      if (point.timestamp_ns < request.header.timestamp_ns || !IsValidPose(point.pose)) {
        *error = "obstacle prediction is not in the planning time frame";
        return false;
      }
    }
  }
  frame->obstacles = request.obstacles;
  return true;
}

bool MapAdapter::Adapt(const PlanningRequest& request, PlanningFrame* frame,
                       std::string* error) const {
  if (request.map.lanes.empty()) {
    *error = "at least one lane is required";
    return false;
  }
  std::unordered_set<std::string> lane_ids;
  for (const Lane& lane : request.map.lanes) {
    if (lane.id.empty() || lane.centerline.size() < 2) {
      *error = "every lane needs an id and two centerline points";
      return false;
    }
    if (!lane_ids.insert(lane.id).second) {
      *error = "lane ids must be unique";
      return false;
    }
    for (size_t index = 0; index < lane.centerline.size(); ++index) {
      const Vec2& point = lane.centerline[index];
      if (!IsFinite(point.x) || !IsFinite(point.y)) {
        *error = "lane centerline points must be finite";
        return false;
      }
      if (index > 0 && Distance(lane.centerline[index - 1], point) < 1e-6) {
        *error = "lane centerline segments must have positive length";
        return false;
      }
    }
  }
  for (const Lane& lane : request.map.lanes) {
    for (const std::string& successor_id : lane.successor_ids) {
      if (!lane_ids.contains(successor_id)) {
        *error = "lane successor id does not exist";
        return false;
      }
    }
  }
  std::unordered_set<std::string> parking_spot_ids;
  for (const ParkingSpot& spot : request.map.parking_spots) {
    if (spot.id.empty() || !parking_spot_ids.insert(spot.id).second ||
        !IsValidPose(spot.entry_pose) || !IsValidPose(spot.target_pose)) {
      *error = "parking spot ids must be unique and poses must be finite";
      return false;
    }
  }
  frame->map = &request.map;
  return true;
}

bool TaskAdapter::Adapt(const PlanningRequest& request, PlanningFrame* frame,
                        std::string* error) const {
  if (request.target_parking_spot_id.empty()) {
    *error = "target parking spot is required";
    return false;
  }
  frame->target_parking_spot_id = request.target_parking_spot_id;
  return true;
}

bool PlanningFrameAdapter::Adapt(const PlanningRequest& request, const VehicleConfig& vehicle,
                                 const PlannerConfig& config, PlanningFrame* frame,
                                 std::string* error) const {
  if (frame == nullptr || error == nullptr) {
    return false;
  }
  // 检查车辆配置
  if (!IsFinite(vehicle.length_m) || !IsFinite(vehicle.width_m) ||
      !IsFinite(vehicle.max_speed_mps) || !IsFinite(vehicle.max_acceleration_mps2) ||
      !IsFinite(vehicle.max_deceleration_mps2) || !IsFinite(vehicle.min_jerk_mps3) ||
      !IsFinite(vehicle.max_jerk_mps3) || !IsFinite(vehicle.max_curvature_1pm) ||
      !IsFinite(vehicle.safety_margin_m) || vehicle.length_m <= 0.0 || vehicle.width_m <= 0.0 ||
      vehicle.max_speed_mps <= 0.0 || vehicle.max_acceleration_mps2 <= 0.0 ||
      vehicle.max_deceleration_mps2 <= 0.0 || vehicle.min_jerk_mps3 >= 0.0 ||
      vehicle.max_jerk_mps3 <= 0.0 || vehicle.max_curvature_1pm <= 0.0 ||
      vehicle.safety_margin_m < 0.0) {
    *error = "invalid vehicle configuration";
    return false;
  }
  // 检查规划配置
  if (!IsFinite(config.horizon_s) || !IsFinite(config.time_step_s) ||
      !IsFinite(config.path_step_m) || !IsFinite(config.max_lane_match_distance_m) ||
      !IsFinite(config.max_lane_heading_difference_rad) || !IsFinite(config.jerk_weight) ||
      !IsFinite(config.max_reverse_speed_mps) ||
      !IsFinite(config.gear_shift_stop_speed_mps) || !IsFinite(config.gear_shift_dwell_s) ||
      config.horizon_s < config.time_step_s || config.time_step_s <= 0.0 ||
      config.path_step_m <= 0.0 || config.path_coupling_iterations <= 0 ||
      config.max_lane_match_distance_m <= 0.0 ||
      config.max_lane_heading_difference_rad <= 0.0 ||
      config.max_lane_heading_difference_rad > kPi || config.jerk_weight < 0.0 ||
      config.max_reverse_speed_mps <= 0.0 ||
      config.max_reverse_speed_mps > vehicle.max_speed_mps ||
      config.gear_shift_stop_speed_mps < 0.0 || config.gear_shift_dwell_s < 0.0) {
    *error = "invalid planner configuration";
    return false;
  }
  // 检查底盘状态
  if (std::abs(request.ego.speed_mps) > vehicle.max_speed_mps + 1e-9 ||
      request.ego.acceleration_mps2 > vehicle.max_acceleration_mps2 + 1e-9 ||
      request.ego.acceleration_mps2 < -vehicle.max_deceleration_mps2 - 1e-9) {
    *error = "ego state violates vehicle longitudinal limits";
    return false;
  }
  frame->vehicle = vehicle;
  frame->config = config;
  return localization_adapter_.Adapt(request, frame, error) &&
         perception_adapter_.Adapt(request, frame, error) &&
         map_adapter_.Adapt(request, frame, error) && task_adapter_.Adapt(request, frame, error);
}
}  // namespace avp
