#include "planning/planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace avp {
namespace {
constexpr double kParkingEntryToleranceM = 0.55;
constexpr double kParkingSegmentDeviationM = 1.0;

Vec2 PositionAtS(const std::vector<PathPoint>& path, double s) {
  if (s <= path.front().s) return path.front().position;
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) {
      const double span = path[i].s - path[i - 1].s;
      return Interpolate(path[i - 1].position, path[i].position,
                         span < 1e-6 ? 0.0 : (s - path[i - 1].s) / span);
    }
  }
  return path.back().position;
}

double YawAtS(const std::vector<PathPoint>& path, double s) {
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) {
      const double span = path[i].s - path[i - 1].s;
      const double ratio = span < 1e-6 ? 0.0 : (s - path[i - 1].s) / span;
      return NormalizeAngle(path[i - 1].yaw +
                            ratio * NormalizeAngle(path[i].yaw - path[i - 1].yaw));
    }
  }
  return path.back().yaw;
}

double CurvatureAtS(const std::vector<PathPoint>& path, double s) {
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) return path[i - 1].curvature;
  }
  return path.back().curvature;
}

double PolylineLength(const std::vector<Vec2>& points) {
  double result = 0.0;
  for (size_t index = 1; index < points.size(); ++index) {
    result += Distance(points[index - 1], points[index]);
  }
  return result;
}

double MaximumTravelDistance(const PlanningFrame& frame, double max_speed_mps) {
  const double initial_speed = std::min(std::abs(frame.ego.speed_mps), max_speed_mps);
  const double acceleration = frame.vehicle.max_acceleration_mps2;
  const double time_to_limit = (max_speed_mps - initial_speed) / acceleration;
  if (time_to_limit >= frame.config.horizon_s) {
    return initial_speed * frame.config.horizon_s +
           0.5 * acceleration * frame.config.horizon_s * frame.config.horizon_s;
  }
  return initial_speed * time_to_limit + 0.5 * acceleration * time_to_limit * time_to_limit +
         max_speed_mps * (frame.config.horizon_s - time_to_limit);
}

double LocalPathHorizon(const PlanningFrame& frame) {
  const double braking_distance = frame.vehicle.max_speed_mps * frame.vehicle.max_speed_mps /
                                  (2.0 * frame.vehicle.max_deceleration_mps2);
  const double geometry_buffer =
      0.5 * frame.vehicle.length_m + frame.vehicle.safety_margin_m;
  return MaximumTravelDistance(frame, frame.vehicle.max_speed_mps) + braking_distance +
         geometry_buffer;
}

struct Projection {
  size_t segment_index = 0;
  Vec2 position;
  double distance = std::numeric_limits<double>::infinity();
};

Projection ProjectToPolyline(const std::vector<Vec2>& points, const Vec2& position) {
  Projection best;
  for (size_t index = 1; index < points.size(); ++index) {
    const Vec2 delta{points[index].x - points[index - 1].x,
                     points[index].y - points[index - 1].y};
    const double length_squared = delta.x * delta.x + delta.y * delta.y;
    const Vec2 offset{position.x - points[index - 1].x,
                      position.y - points[index - 1].y};
    const double ratio = std::clamp((offset.x * delta.x + offset.y * delta.y) / length_squared,
                                    0.0, 1.0);
    const Vec2 projected = Interpolate(points[index - 1], points[index], ratio);
    const double distance = Distance(position, projected);
    if (distance < best.distance) {
      best = {index - 1, projected, distance};
    }
  }
  return best;
}

struct CroppedRoute {
  GlobalRoute route;
  double global_length_m = 0.0;
  double local_length_m = 0.0;
  bool reaches_end = false;
};

CroppedRoute CropRoute(const GlobalRoute& route, const Vec2& ego_position, double horizon_m) {
  CroppedRoute result;
  result.route = route;
  result.route.reference_line.clear();
  result.global_length_m = PolylineLength(route.reference_line);
  const Projection projection = ProjectToPolyline(route.reference_line, ego_position);
  result.route.reference_line.push_back(projection.position);

  Vec2 current = projection.position;
  double remaining = horizon_m;
  for (size_t index = projection.segment_index + 1; index < route.reference_line.size(); ++index) {
    const Vec2 end = route.reference_line[index];
    const double length = Distance(current, end);
    if (length < 1e-9) {
      current = end;
      continue;
    }
    if (length <= remaining + 1e-9) {
      result.route.reference_line.push_back(end);
      remaining -= length;
      current = end;
      continue;
    }
    result.route.reference_line.push_back(Interpolate(current, end, remaining / length));
    remaining = 0.0;
    break;
  }
  result.reaches_end =
      Distance(result.route.reference_line.back(), route.reference_line.back()) < 1e-6;
  result.local_length_m = PolylineLength(result.route.reference_line);
  return result;
}

std::vector<PathPoint> DensifyPathForSpeed(const PlanningFrame& frame,
                                           const std::vector<PathPoint>& path,
                                           double max_speed_mps) {
  std::vector<PathPoint> result;
  result.push_back(path.front());
  result.front().s = 0.0;
  const double fine_step = std::max(
      0.005, std::min(0.02, frame.vehicle.max_jerk_mps3 *
                                std::pow(frame.config.time_step_s, 3.0) * 0.5));
  const double coarse_step =
      std::max(fine_step, std::min(0.05, max_speed_mps * frame.config.time_step_s * 0.25));
  const double total_length = path.back().s;
  for (double s = fine_step; s < total_length - 1e-9;) {
    result.push_back({PositionAtS(path, s), YawAtS(path, s), CurvatureAtS(path, s), s});
    s += s < 0.25 ? fine_step : coarse_step;
  }
  if (total_length > 1e-9) {
    result.push_back(path.back());
  }
  return result;
}

PredictionPoint PredictAt(const Obstacle& obstacle, uint64_t timestamp_ns) {
  PredictionPoint result = obstacle.prediction.front();
  for (const PredictionPoint& point : obstacle.prediction) {
    if (point.timestamp_ns > timestamp_ns) break;
    result = point;
  }
  return result;
}

bool IsCollisionFree(const PlanningFrame& frame,
                     const std::vector<TimedTrajectoryPoint>& trajectory) {
  for (const TimedTrajectoryPoint& point : trajectory) {
    const uint64_t timestamp =
        frame.header.timestamp_ns + static_cast<uint64_t>(point.relative_time_s * 1e9);
    for (const Obstacle& obstacle : frame.obstacles) {
      if (IsVehicleObstacleCollision(point.pose, frame.vehicle, PredictAt(obstacle, timestamp).pose,
                                     obstacle.length_m, obstacle.width_m)) {
        return false;
      }
    }
  }
  return true;
}

std::vector<TimedTrajectoryPoint> MakeStopTrajectory(const PlanningFrame& frame) {
  std::vector<TimedTrajectoryPoint> result;
  const DrivingDirection direction = frame.ego.direction == DrivingDirection::kReverse
                                         ? DrivingDirection::kReverse
                                         : DrivingDirection::kDrive;
  const double direction_sign = direction == DrivingDirection::kReverse ? -1.0 : 1.0;
  const double initial_speed = std::abs(frame.ego.speed_mps);
  const double deceleration = std::max(0.1, frame.vehicle.max_deceleration_mps2);
  const double stop_time = std::max(frame.config.time_step_s, initial_speed / deceleration);
  for (double t = 0.0; t <= stop_time + 1e-9; t += frame.config.time_step_s) {
    const double speed = std::max(0.0, initial_speed - deceleration * t);
    const double distance = std::max(0.0, initial_speed * t - 0.5 * deceleration * t * t);
    const Vec2 position{frame.ego.pose.position.x +
                            direction_sign * std::cos(frame.ego.pose.yaw) * distance,
                        frame.ego.pose.position.y +
                            direction_sign * std::sin(frame.ego.pose.yaw) * distance};
    result.push_back({{position, frame.ego.pose.yaw}, 0.0, speed, -deceleration, t, direction});
  }
  return result;
}

std::vector<TimedTrajectoryPoint> MakeStationaryTrajectory(const PlanningFrame& frame,
                                                            DrivingDirection direction) {
  std::vector<TimedTrajectoryPoint> result;
  const int count =
      static_cast<int>(std::floor(frame.config.horizon_s / frame.config.time_step_s)) + 1;
  result.reserve(count);
  for (int index = 0; index < count; ++index) {
    result.push_back({frame.ego.pose, 0.0, 0.0, 0.0, index * frame.config.time_step_s, direction});
  }
  return result;
}

PlanningResponse MakeFallbackResponse(const PlanningFrame& frame, const std::string& message,
                                      const std::string& mode) {
  PlanningResponse response;
  response.header = frame.header;
  response.trajectory = MakeStopTrajectory(frame);
  response.status = PlanningStatus::kNoSafeTrajectory;
  response.message = message;
  response.diagnostics.push_back("planning_mode=" + mode);
  response.diagnostics.push_back("fallback=emergency_stop");
  response.diagnostics.push_back("jerk_constraint=emergency_exempt");
  response.diagnostics.push_back(IsCollisionFree(frame, response.trajectory)
                                     ? "stop_collision_free=true"
                                     : "stop_collision_free=false");
  return response;
}

std::vector<PathPoint> BuildParkingPath(const PlanningFrame& frame,
                                        const ParkingSegment& segment, double max_length_m,
                                        double max_speed_mps, double* nearest_distance_m,
                                        bool* reaches_end) {
  size_t nearest = 0;
  *nearest_distance_m = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < segment.points.size(); ++index) {
    const double distance = Distance(frame.ego.pose.position, segment.points[index].pose.position);
    if (distance < *nearest_distance_m) {
      *nearest_distance_m = distance;
      nearest = index;
    }
  }

  std::vector<PathPoint> coarse;
  coarse.push_back({frame.ego.pose.position, frame.ego.pose.yaw,
                    segment.points[nearest].signed_curvature_1pm, 0.0});
  double s = 0.0;
  for (size_t index = nearest + 1; index < segment.points.size(); ++index) {
    const HybridPathPoint& point = segment.points[index];
    const double length = Distance(coarse.back().position, point.pose.position);
    if (length < 1e-9) continue;
    if (s + length > max_length_m) {
      const double remaining = max_length_m - s;
      const double ratio = remaining / length;
      const double yaw = NormalizeAngle(coarse.back().yaw +
                                        ratio * NormalizeAngle(point.pose.yaw - coarse.back().yaw));
      coarse.push_back({Interpolate(coarse.back().position, point.pose.position, ratio), yaw,
                        point.signed_curvature_1pm, max_length_m});
      *reaches_end = false;
      return DensifyPathForSpeed(frame, coarse, max_speed_mps);
    }
    s += length;
    coarse.push_back(
        {point.pose.position, point.pose.yaw, point.signed_curvature_1pm, s});
  }
  *reaches_end = true;
  return DensifyPathForSpeed(frame, coarse, max_speed_mps);
}

std::vector<TimedTrajectoryPoint> ComposeTrajectory(const std::vector<PathPoint>& path,
                                                    const std::vector<SpeedPoint>& speed,
                                                    DrivingDirection direction) {
  std::vector<TimedTrajectoryPoint> result;
  result.reserve(speed.size());
  for (const SpeedPoint& point : speed) {
    result.push_back({{PositionAtS(path, point.s), YawAtS(path, point.s)},
                      CurvatureAtS(path, point.s), point.speed_mps, point.acceleration_mps2,
                      point.time_s, direction});
  }
  return result;
}

std::string Metric(const char* name, double value) {
  std::ostringstream stream;
  stream << name << '=' << value;
  return stream.str();
}
}  // namespace

Planner::Planner(VehicleConfig vehicle, PlannerConfig config)
    : vehicle_(vehicle), config_(config) {}

void Planner::ResetTask(const std::string& target_parking_spot_id) {
  active_target_parking_spot_id_ = target_parking_spot_id;
  mode_ = Mode::kLaneApproach;
  parking_maneuver_.segments.clear();
  parking_segment_index_ = 0;
  gear_shift_start_timestamp_ns_ = 0;
  last_parking_replan_timestamp_ns_ = 0;
  pending_direction_ = DrivingDirection::kUnknown;
}

PlanningResponse Planner::PlanParking(const PlanningFrame& frame) {
  if (parking_segment_index_ >= parking_maneuver_.segments.size()) {
    PlanningResponse response;
    response.header = frame.header;
    response.status = PlanningStatus::kOk;
    response.message = "parking maneuver completed";
    response.trajectory = MakeStationaryTrajectory(frame, frame.ego.direction);
    response.diagnostics = {"planning_mode=OPEN_SPACE_PARKING", "parking_complete=true"};
    return response;
  }

  const ParkingSegment& segment = parking_maneuver_.segments[parking_segment_index_];
  if (mode_ == Mode::kGearShift) {
    const double elapsed_s = frame.header.timestamp_ns >= gear_shift_start_timestamp_ns_
                                 ? static_cast<double>(frame.header.timestamp_ns -
                                                       gear_shift_start_timestamp_ns_) /
                                       1e9
                                 : 0.0;
    if (std::abs(frame.ego.speed_mps) > frame.config.gear_shift_stop_speed_mps ||
        elapsed_s < frame.config.gear_shift_dwell_s ||
        frame.ego.direction != pending_direction_) {
      PlanningResponse response;
      response.header = frame.header;
      response.status = PlanningStatus::kOk;
      response.message = "waiting for safe gear shift";
      response.trajectory = MakeStationaryTrajectory(frame, pending_direction_);
      response.diagnostics = {"planning_mode=GEAR_SHIFT",
                              std::string("gear=") + ToString(pending_direction_),
                              "parking_segment_index=" +
                                  std::to_string(parking_segment_index_)};
      return response;
    }
    mode_ = Mode::kOpenSpaceParking;
  }

  if (frame.ego.direction != segment.direction) {
    mode_ = Mode::kGearShift;
    pending_direction_ = segment.direction;
    gear_shift_start_timestamp_ns_ = frame.header.timestamp_ns;
    return PlanParking(frame);
  }

  const double max_speed = segment.direction == DrivingDirection::kReverse
                               ? frame.config.max_reverse_speed_mps
                               : frame.vehicle.max_speed_mps;
  const double max_path_length = MaximumTravelDistance(frame, max_speed);
  double nearest_distance = 0.0;
  bool reaches_end = false;
  std::vector<PathPoint> path =
      BuildParkingPath(frame, segment, max_path_length, max_speed, &nearest_distance,
                       &reaches_end);
  if (nearest_distance > kParkingSegmentDeviationM) {
    std::string error;
    ParkingManeuver replanned;
    const ParkingSpot* spot = nullptr;
    for (const ParkingSpot& candidate : frame.map->parking_spots) {
      if (candidate.id == frame.target_parking_spot_id) {
        spot = &candidate;
        break;
      }
    }
    if (spot == nullptr ||
        !hybrid_a_star_.Plan(frame, frame.ego.pose, spot->target_pose, &replanned, &error)) {
      return MakeFallbackResponse(frame, error.empty() ? "parking replan failed" : error,
                                  "OPEN_SPACE_PARKING");
    }
    parking_maneuver_ = std::move(replanned);
    parking_segment_index_ = 0;
    return PlanParking(frame);
  }

  if (path.size() < 2 ||
      (reaches_end && Distance(frame.ego.pose.position, segment.points.back().pose.position) <
                          kParkingEntryToleranceM &&
       std::abs(frame.ego.speed_mps) <= frame.config.gear_shift_stop_speed_mps)) {
    if (parking_segment_index_ + 1 >= parking_maneuver_.segments.size()) {
      ++parking_segment_index_;
      return PlanParking(frame);
    }
    ++parking_segment_index_;
    pending_direction_ = parking_maneuver_.segments[parking_segment_index_].direction;
    mode_ = Mode::kGearShift;
    gear_shift_start_timestamp_ns_ = frame.header.timestamp_ns;
    return PlanParking(frame);
  }

  std::vector<SpeedPoint> speed;
  std::string error;
  if (!speed_planner_.Plan(frame, path, &speed, &error, {max_speed, false})) {
    return MakeFallbackResponse(frame, error, "OPEN_SPACE_PARKING");
  }
  if (reaches_end && speed.back().s >= path.back().s - 0.1) {
    std::vector<SpeedPoint> stopped_speed;
    std::string stop_error;
    if (speed_planner_.Plan(frame, path, &stopped_speed, &stop_error, {max_speed, true})) {
      speed = std::move(stopped_speed);
    }
  }

  PlanningResponse response;
  response.header = frame.header;
  response.trajectory = ComposeTrajectory(path, speed, segment.direction);
  if (!IsCollisionFree(frame, response.trajectory)) {
    if (last_parking_replan_timestamp_ns_ != frame.header.timestamp_ns) {
      const ParkingSpot* spot = nullptr;
      for (const ParkingSpot& candidate : frame.map->parking_spots) {
        if (candidate.id == frame.target_parking_spot_id) {
          spot = &candidate;
          break;
        }
      }
      ParkingManeuver replanned;
      std::string replan_error;
      last_parking_replan_timestamp_ns_ = frame.header.timestamp_ns;
      if (spot != nullptr && hybrid_a_star_.Plan(frame, frame.ego.pose, spot->target_pose,
                                                 &replanned, &replan_error)) {
        parking_maneuver_ = std::move(replanned);
        parking_segment_index_ = 0;
        mode_ = Mode::kOpenSpaceParking;
        return PlanParking(frame);
      }
    }
    return MakeFallbackResponse(frame, "post-plan parking collision validation failed",
                                "OPEN_SPACE_PARKING");
  }
  response.status = PlanningStatus::kOk;
  response.message = "open-space parking trajectory generated";
  response.diagnostics = {"planning_mode=OPEN_SPACE_PARKING",
                          std::string("gear=") + ToString(segment.direction),
                          "parking_segment_index=" + std::to_string(parking_segment_index_),
                          "speed=ST_DP"};
  return response;
}

PlanningResponse Planner::Plan(const PlanningRequest& request) {
  PlanningResponse response;
  response.header = request.header;

  PlanningFrame frame;
  std::string error;
  if (!adapter_.Adapt(request, vehicle_, config_, &frame, &error)) {
    response.status = PlanningStatus::kInvalidInput;
    response.message = error;
    return response;
  }
  if (active_target_parking_spot_id_ != frame.target_parking_spot_id) {
    ResetTask(frame.target_parking_spot_id);
  }
  if (mode_ != Mode::kLaneApproach) {
    return PlanParking(frame);
  }

  GlobalRoute route;
  if (!global_planner_.Plan(frame, &route, &error)) {
    response.status = PlanningStatus::kNoRoute;
    response.message = error;
    return response;
  }

  if (Distance(frame.ego.pose.position, route.parking_entry.position) <=
          kParkingEntryToleranceM &&
      std::abs(frame.ego.speed_mps) <= frame.config.gear_shift_stop_speed_mps) {
    if (Distance(frame.ego.pose.position, route.parking_target.position) <=
            kParkingEntryToleranceM &&
        std::abs(NormalizeAngle(frame.ego.pose.yaw - route.parking_target.yaw)) < 0.45) {
      parking_segment_index_ = 0;
      parking_maneuver_.segments.clear();
      return PlanParking(frame);
    }
    if (!hybrid_a_star_.Plan(frame, frame.ego.pose, route.parking_target, &parking_maneuver_,
                             &error)) {
      return MakeFallbackResponse(frame, error, "OPEN_SPACE_PARKING");
    }
    parking_segment_index_ = 0;
    const DrivingDirection first_direction = parking_maneuver_.segments.front().direction;
    if (frame.ego.direction == first_direction) {
      mode_ = Mode::kOpenSpaceParking;
    } else {
      mode_ = Mode::kGearShift;
      pending_direction_ = first_direction;
      gear_shift_start_timestamp_ns_ = frame.header.timestamp_ns;
    }
    return PlanParking(frame);
  }

  const double local_horizon = LocalPathHorizon(frame);
  const CroppedRoute cropped = CropRoute(route, frame.ego.pose.position, local_horizon);
  if (cropped.route.reference_line.size() < 2) {
    if (Distance(frame.ego.pose.position, route.parking_entry.position) <=
        kParkingEntryToleranceM) {
      response.status = PlanningStatus::kOk;
      response.message = "stopping at parking entry before mode transition";
      response.trajectory = MakeStopTrajectory(frame);
      response.diagnostics = {"planning_mode=LANE_APPROACH", "parking_entry_stop=true",
                              "gear=DRIVE"};
      return response;
    }
    return MakeFallbackResponse(frame, "local route has no usable length", "LANE_APPROACH");
  }

  std::vector<double> arrivals;
  std::vector<PathPoint> path;
  std::vector<PathPoint> speed_path;
  std::vector<SpeedPoint> speed;
  for (int iteration = 0; iteration < frame.config.path_coupling_iterations; ++iteration) {
    std::vector<PathPoint> candidate_path;
    std::vector<SpeedPoint> candidate_speed;
    if (!local_planner_.Plan(frame, cropped.route, arrivals, &candidate_path, &error)) break;
    std::vector<PathPoint> candidate_speed_path =
        DensifyPathForSpeed(frame, candidate_path, frame.vehicle.max_speed_mps);
    if (!speed_planner_.Plan(frame, candidate_speed_path, &candidate_speed, &error,
                             {frame.vehicle.max_speed_mps, false})) {
      break;
    }
    if (cropped.reaches_end &&
        candidate_speed.back().s >= candidate_speed_path.back().s - 0.1) {
      std::vector<SpeedPoint> stopped_speed;
      std::string stop_error;
      if (speed_planner_.Plan(frame, candidate_speed_path, &stopped_speed, &stop_error,
                              {frame.vehicle.max_speed_mps, true})) {
        candidate_speed = std::move(stopped_speed);
      }
    }
    path = std::move(candidate_path);
    speed_path = std::move(candidate_speed_path);
    speed = std::move(candidate_speed);

    arrivals.assign(path.size(), frame.config.horizon_s);
    for (size_t index = 0; index < path.size(); ++index) {
      for (const SpeedPoint& point : speed) {
        if (point.s + 1e-6 >= path[index].s) {
          arrivals[index] = point.time_s;
          break;
        }
      }
    }
  }

  if (path.empty() || speed_path.empty() || speed.empty()) {
    return MakeFallbackResponse(
        frame, error.empty() ? "no feasible local path or speed profile" : error,
        "LANE_APPROACH");
  }

  response.trajectory = ComposeTrajectory(speed_path, speed, DrivingDirection::kDrive);
  if (!IsCollisionFree(frame, response.trajectory)) {
    return MakeFallbackResponse(frame, "post-plan collision validation failed", "LANE_APPROACH");
  }

  response.status = PlanningStatus::kOk;
  response.message = "rolling-horizon spatiotemporal DP trajectory generated";
  response.diagnostics = {"planning_mode=LANE_APPROACH", "global=A_STAR", "local=SL_DP",
                          "speed=ST_DP", "gear=DRIVE",
                          Metric("global_route_length_m", cropped.global_length_m),
                          Metric("local_horizon_m", cropped.local_length_m),
                          "path_layer_count=" + std::to_string(path.size())};
  return response;
}
}  // namespace avp
