#ifndef AVP_COMMON_TYPES_H_
#define AVP_COMMON_TYPES_H_

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace avp {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};
// 计算两点间的欧几里得距离。
inline double Distance(const Vec2& first, const Vec2& second) {
  return std::hypot(first.x - second.x, first.y - second.y);
}
// 将角度归一化到 (-pi, pi]。
inline double NormalizeAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle <= -kPi) angle += 2.0 * kPi;
  return angle;
}
inline Vec2 Interpolate(const Vec2& first, const Vec2& second, double ratio) {
  return {first.x + (second.x - first.x) * ratio, first.y + (second.y - first.y) * ratio};
}

struct Pose2d {
  Vec2 position;
  double yaw = 0.0;
};
enum class DrivingDirection { kUnknown = 0, kDrive = 1, kReverse = 2 };

inline const char* ToString(DrivingDirection direction) {
  switch (direction) {
    case DrivingDirection::kDrive:
      return "DRIVE";
    case DrivingDirection::kReverse:
      return "REVERSE";
    case DrivingDirection::kUnknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}
struct Header {
  std::string frame_id;         
  uint64_t timestamp_ns = 0;    // 当前帧时间戳，单位纳秒
  uint64_t sequence_id = 0;     
};
struct EgoState {
  Pose2d pose;
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
  DrivingDirection direction = DrivingDirection::kDrive;
};
struct PredictionPoint {
  uint64_t timestamp_ns = 0;
  Pose2d pose;
  double speed_mps = 0.0;
};
struct Obstacle {
  std::string id;
  double length_m = 0.0;
  double width_m = 0.0;
  double confidence = 1.0;
  std::vector<PredictionPoint> prediction;
};
struct Lane {
  std::string id;   // 车道 ID
  std::vector<Vec2> centerline; // 车道中心线点
  std::vector<std::string> successor_ids; // 后续车道 ID
  bool closed = false;  // 车道是否关闭
};
struct ParkingSpot {
  std::string id;
  Pose2d entry_pose;
  Pose2d target_pose;
};
struct MapSnapshot {
  std::vector<Lane> lanes;
  std::vector<ParkingSpot> parking_spots;
};

struct VehicleConfig {
  double length_m = 4.8;
  double width_m = 2.0;
  double wheelbase_m = 2.8;
  double max_speed_mps = 3.0;
  double max_acceleration_mps2 = 1.5;
  double max_deceleration_mps2 = 2.0;
  double max_curvature_1pm = 0.25;
  double safety_margin_m = 0.35;
  double min_jerk_mps3 = -4.0;
  double max_jerk_mps3 = 2.0;
};

inline double Dot(const Vec2& first, const Vec2& second) {
  return first.x * second.x + first.y * second.y;
}

inline Vec2 HeadingAxis(double yaw) { return {std::cos(yaw), std::sin(yaw)}; }

inline Vec2 LateralAxis(double yaw) { return {-std::sin(yaw), std::cos(yaw)}; }

// 使用分离轴定理检测两个有向矩形。自车半尺寸按安全边距膨胀，边界接触视为碰撞。
inline bool IsVehicleObstacleCollision(const Pose2d& vehicle_pose, const VehicleConfig& vehicle,
                                       const Pose2d& obstacle_pose, double obstacle_length_m,
                                       double obstacle_width_m) {
  const double vehicle_half_length = vehicle.length_m * 0.5 + vehicle.safety_margin_m;
  const double vehicle_half_width = vehicle.width_m * 0.5 + vehicle.safety_margin_m;
  const double obstacle_half_length = obstacle_length_m * 0.5;
  const double obstacle_half_width = obstacle_width_m * 0.5;
  const std::array<Vec2, 2> vehicle_axes = {HeadingAxis(vehicle_pose.yaw),
                                            LateralAxis(vehicle_pose.yaw)};
  const std::array<Vec2, 2> obstacle_axes = {HeadingAxis(obstacle_pose.yaw),
                                             LateralAxis(obstacle_pose.yaw)};
  const Vec2 center_delta{obstacle_pose.position.x - vehicle_pose.position.x,
                          obstacle_pose.position.y - vehicle_pose.position.y};

  for (const Vec2& axis : {vehicle_axes[0], vehicle_axes[1], obstacle_axes[0], obstacle_axes[1]}) {
    const double vehicle_projection = vehicle_half_length * std::abs(Dot(vehicle_axes[0], axis)) +
                                      vehicle_half_width * std::abs(Dot(vehicle_axes[1], axis));
    const double obstacle_projection =
        obstacle_half_length * std::abs(Dot(obstacle_axes[0], axis)) +
        obstacle_half_width * std::abs(Dot(obstacle_axes[1], axis));
    if (std::abs(Dot(center_delta, axis)) > vehicle_projection + obstacle_projection) {
      return false;
    }
  }
  return true;
}

struct PlannerConfig {
  double horizon_s = 8.0;
  double time_step_s = 0.2;
  double path_step_m = 0.5;
  double input_max_age_s = 0.5;
  int path_coupling_iterations = 2;
  double max_lane_match_distance_m = 2.0;               // 点离车道中心线多远仍可认为属于该车道，默认 2 m
  double max_lane_heading_difference_rad = kPi / 3.0;   // 自车朝向和车道方向允许的最大差异，默认 π/3
  double jerk_weight = 1.0;
  double max_reverse_speed_mps = 1.0;
  double gear_shift_stop_speed_mps = 0.05;
  double gear_shift_dwell_s = 1.0;
};
struct PlanningRequest {
  Header header;
  EgoState ego;
  std::string target_parking_spot_id;
  MapSnapshot map;
  std::vector<Obstacle> obstacles;
};
struct PlanningFrame {
  Header header;
  EgoState ego;
  const MapSnapshot* map = nullptr;
  std::string target_parking_spot_id;
  std::vector<Obstacle> obstacles;
  VehicleConfig vehicle;
  PlannerConfig config;
};
struct PathPoint {
  Vec2 position;
  double yaw = 0.0;
  double curvature = 0.0;
  double s = 0.0;
};
struct TimedTrajectoryPoint {
  Pose2d pose;
  double curvature_1pm = 0.0;
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
  double relative_time_s = 0.0;
  DrivingDirection direction = DrivingDirection::kDrive;
};

enum class PlanningStatus { kOk, kInvalidInput, kNoRoute, kNoSafeTrajectory, kInternalError };
struct PlanningResponse {
  Header header;
  PlanningStatus status = PlanningStatus::kInternalError;
  std::string message;
  std::vector<TimedTrajectoryPoint> trajectory;
  std::vector<std::string> diagnostics;
};

inline const char* ToString(PlanningStatus status) {
  switch (status) {
    case PlanningStatus::kOk:
      return "OK";
    case PlanningStatus::kInvalidInput:
      return "INVALID_INPUT";
    case PlanningStatus::kNoRoute:
      return "NO_ROUTE";
    case PlanningStatus::kNoSafeTrajectory:
      return "NO_SAFE_TRAJECTORY";
    case PlanningStatus::kInternalError:
      return "INTERNAL_ERROR";
  }
  return "INTERNAL_ERROR";
}
}  // namespace avp
#endif  // 头文件保护
