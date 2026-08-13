#include "tools/scenario.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace avp::tools {
namespace {

QJsonObject PoseToJson(const Pose2d& pose) {
  return {{"x", pose.position.x}, {"y", pose.position.y}, {"yaw", pose.yaw}};
}

bool PoseFromJson(const QJsonObject& object, Pose2d* pose) {
  if (!object.contains("x") || !object.contains("y") || !object.contains("yaw")) return false;
  pose->position = {object["x"].toDouble(), object["y"].toDouble()};
  pose->yaw = object["yaw"].toDouble();
  return std::isfinite(pose->position.x) && std::isfinite(pose->position.y) &&
         std::isfinite(pose->yaw);
}

QJsonObject EgoToJson(const EgoState& ego) {
  return {{"pose", PoseToJson(ego.pose)},
          {"speed_mps", ego.speed_mps},
          {"acceleration_mps2", ego.acceleration_mps2},
          {"direction", static_cast<int>(ego.direction)}};
}

bool EgoFromJson(const QJsonObject& object, EgoState* ego) {
  if (!PoseFromJson(object["pose"].toObject(), &ego->pose)) return false;
  ego->speed_mps = object["speed_mps"].toDouble();
  ego->acceleration_mps2 = object["acceleration_mps2"].toDouble();
  ego->direction = static_cast<DrivingDirection>(object["direction"].toInt(1));
  return std::isfinite(ego->speed_mps) && std::isfinite(ego->acceleration_mps2);
}

QJsonObject VehicleToJson(const VehicleConfig& vehicle) {
  return {{"length_m", vehicle.length_m}, {"width_m", vehicle.width_m},
          {"wheelbase_m", vehicle.wheelbase_m}, {"max_speed_mps", vehicle.max_speed_mps},
          {"max_acceleration_mps2", vehicle.max_acceleration_mps2},
          {"max_deceleration_mps2", vehicle.max_deceleration_mps2},
          {"max_curvature_1pm", vehicle.max_curvature_1pm},
          {"safety_margin_m", vehicle.safety_margin_m}, {"min_jerk_mps3", vehicle.min_jerk_mps3},
          {"max_jerk_mps3", vehicle.max_jerk_mps3}};
}

void VehicleFromJson(const QJsonObject& object, VehicleConfig* vehicle) {
  if (object.isEmpty()) return;
  vehicle->length_m = object["length_m"].toDouble(vehicle->length_m);
  vehicle->width_m = object["width_m"].toDouble(vehicle->width_m);
  vehicle->wheelbase_m = object["wheelbase_m"].toDouble(vehicle->wheelbase_m);
  vehicle->max_speed_mps = object["max_speed_mps"].toDouble(vehicle->max_speed_mps);
  vehicle->max_acceleration_mps2 =
      object["max_acceleration_mps2"].toDouble(vehicle->max_acceleration_mps2);
  vehicle->max_deceleration_mps2 =
      object["max_deceleration_mps2"].toDouble(vehicle->max_deceleration_mps2);
  vehicle->max_curvature_1pm =
      object["max_curvature_1pm"].toDouble(vehicle->max_curvature_1pm);
  vehicle->safety_margin_m = object["safety_margin_m"].toDouble(vehicle->safety_margin_m);
  vehicle->min_jerk_mps3 = object["min_jerk_mps3"].toDouble(vehicle->min_jerk_mps3);
  vehicle->max_jerk_mps3 = object["max_jerk_mps3"].toDouble(vehicle->max_jerk_mps3);
}

QJsonObject PlannerToJson(const PlannerConfig& config) {
  return {{"horizon_s", config.horizon_s}, {"time_step_s", config.time_step_s},
          {"path_step_m", config.path_step_m}, {"input_max_age_s", config.input_max_age_s},
          {"path_coupling_iterations", config.path_coupling_iterations},
          {"max_lane_match_distance_m", config.max_lane_match_distance_m},
          {"max_lane_heading_difference_rad", config.max_lane_heading_difference_rad},
          {"jerk_weight", config.jerk_weight}, {"max_reverse_speed_mps", config.max_reverse_speed_mps},
          {"gear_shift_stop_speed_mps", config.gear_shift_stop_speed_mps},
          {"gear_shift_dwell_s", config.gear_shift_dwell_s}};
}

void PlannerFromJson(const QJsonObject& object, PlannerConfig* config) {
  if (object.isEmpty()) return;
  config->horizon_s = object["horizon_s"].toDouble(config->horizon_s);
  config->time_step_s = object["time_step_s"].toDouble(config->time_step_s);
  config->path_step_m = object["path_step_m"].toDouble(config->path_step_m);
  config->input_max_age_s = object["input_max_age_s"].toDouble(config->input_max_age_s);
  config->path_coupling_iterations =
      object["path_coupling_iterations"].toInt(config->path_coupling_iterations);
  config->max_lane_match_distance_m =
      object["max_lane_match_distance_m"].toDouble(config->max_lane_match_distance_m);
  config->max_lane_heading_difference_rad = object["max_lane_heading_difference_rad"].toDouble(
      config->max_lane_heading_difference_rad);
  config->jerk_weight = object["jerk_weight"].toDouble(config->jerk_weight);
  config->max_reverse_speed_mps =
      object["max_reverse_speed_mps"].toDouble(config->max_reverse_speed_mps);
  config->gear_shift_stop_speed_mps =
      object["gear_shift_stop_speed_mps"].toDouble(config->gear_shift_stop_speed_mps);
  config->gear_shift_dwell_s = object["gear_shift_dwell_s"].toDouble(config->gear_shift_dwell_s);
}

double LoopTime(const ScenarioObstacle& obstacle, double time_s) {
  if (!obstacle.loop || obstacle.keyframes.size() < 2) return time_s;
  const double start = obstacle.keyframes.front().time_s;
  const double period = obstacle.keyframes.back().time_s - start;
  if (period <= 1e-9) return time_s;
  return start + std::fmod(std::fmod(time_s - start, period) + period, period);
}

const ObstacleKeyframe& FirstOrDefault(const ScenarioObstacle& obstacle) {
  static const ObstacleKeyframe kDefault;
  return obstacle.keyframes.empty() ? kDefault : obstacle.keyframes.front();
}

}  // namespace

SimulationScenario MakeDefaultScenario() {
  SimulationScenario scenario;
  scenario.map.lanes = {{"entry", {{0.0, 0.0}, {8.0, 0.0}}, {"parking_lane"}, false},
                        {"parking_lane", {{8.0, 0.0}, {12.0, 0.0}}, {}, false}};
  scenario.map.parking_spots = {{"P1", {{12.0, 0.0}, 0.0}, {{13.0, 0.0}, 0.0}}};
  scenario.initial_ego.pose = {{0.0, 0.0}, 0.0};
  scenario.initial_ego.direction = DrivingDirection::kDrive;
  scenario.target_parking_spot_id = "P1";
  return scenario;
}

Pose2d SampleObstaclePose(const ScenarioObstacle& obstacle, double time_s) {
  const auto& first = FirstOrDefault(obstacle);
  if (obstacle.keyframes.size() < 2) return first.pose;
  const double sample_time = LoopTime(obstacle, time_s);
  if (sample_time <= first.time_s) return first.pose;
  for (size_t index = 1; index < obstacle.keyframes.size(); ++index) {
    const ObstacleKeyframe& next = obstacle.keyframes[index];
    if (sample_time <= next.time_s) {
      const ObstacleKeyframe& previous = obstacle.keyframes[index - 1];
      const double span = next.time_s - previous.time_s;
      const double ratio = span <= 1e-9 ? 0.0 : (sample_time - previous.time_s) / span;
      return {Interpolate(previous.pose.position, next.pose.position, ratio),
              NormalizeAngle(previous.pose.yaw +
                             ratio * NormalizeAngle(next.pose.yaw - previous.pose.yaw))};
    }
  }
  return obstacle.keyframes.back().pose;
}

double SampleObstacleSpeed(const ScenarioObstacle& obstacle, double time_s) {
  const auto& first = FirstOrDefault(obstacle);
  if (obstacle.keyframes.size() < 2) return first.speed_mps;
  const double sample_time = LoopTime(obstacle, time_s);
  if (sample_time <= first.time_s) return first.speed_mps;
  for (size_t index = 1; index < obstacle.keyframes.size(); ++index) {
    const ObstacleKeyframe& next = obstacle.keyframes[index];
    if (sample_time <= next.time_s) {
      const ObstacleKeyframe& previous = obstacle.keyframes[index - 1];
      const double ratio = (sample_time - previous.time_s) / (next.time_s - previous.time_s);
      return previous.speed_mps + ratio * (next.speed_mps - previous.speed_mps);
    }
  }
  return obstacle.keyframes.back().speed_mps;
}

PlanningRequest MakePlanningRequest(const SimulationScenario& scenario, const EgoState& ego,
                                    double simulation_time_s, uint64_t sequence_id) {
  PlanningRequest request;
  constexpr uint64_t kSimulationEpochNs = 1'000'000'000;
  request.header = {"map", kSimulationEpochNs +
                                static_cast<uint64_t>(std::llround(simulation_time_s * 1e9)),
                    sequence_id};
  request.ego = ego;
  request.target_parking_spot_id = scenario.target_parking_spot_id;
  request.map = scenario.map;
  const int count = static_cast<int>(std::ceil(scenario.planner.horizon_s / scenario.planner.time_step_s));
  for (const ScenarioObstacle& source : scenario.obstacles) {
    Obstacle obstacle{source.id, source.length_m, source.width_m, source.confidence, {}};
    for (int index = 0; index <= count; ++index) {
      const double relative_time_s = index * scenario.planner.time_step_s;
      obstacle.prediction.push_back(
          {request.header.timestamp_ns + static_cast<uint64_t>(std::llround(relative_time_s * 1e9)),
           SampleObstaclePose(source, simulation_time_s + relative_time_s),
           SampleObstacleSpeed(source, simulation_time_s + relative_time_s)});
    }
    request.obstacles.push_back(std::move(obstacle));
  }
  return request;
}

bool SaveScenarioJson(const SimulationScenario& scenario, const std::string& path, std::string* error) {
  QJsonObject root{{"schema_version", scenario.schema_version},
                   {"initial_ego", EgoToJson(scenario.initial_ego)},
                   {"target_parking_spot_id", QString::fromStdString(scenario.target_parking_spot_id)},
                   {"vehicle", VehicleToJson(scenario.vehicle)}, {"planner", PlannerToJson(scenario.planner)}};
  QJsonArray lanes;
  for (const Lane& lane : scenario.map.lanes) {
    QJsonArray centerline;
    for (const Vec2& point : lane.centerline) centerline.append(QJsonArray{point.x, point.y});
    QJsonArray successors;
    for (const std::string& successor : lane.successor_ids) successors.append(QString::fromStdString(successor));
    lanes.append(QJsonObject{{"id", QString::fromStdString(lane.id)}, {"centerline", centerline},
                             {"successor_ids", successors}, {"closed", lane.closed}});
  }
  root["lanes"] = lanes;
  QJsonArray spots;
  for (const ParkingSpot& spot : scenario.map.parking_spots) {
    spots.append(QJsonObject{{"id", QString::fromStdString(spot.id)}, {"entry_pose", PoseToJson(spot.entry_pose)},
                             {"target_pose", PoseToJson(spot.target_pose)}});
  }
  root["parking_spots"] = spots;
  QJsonArray obstacles;
  for (const ScenarioObstacle& obstacle : scenario.obstacles) {
    QJsonArray keyframes;
    for (const ObstacleKeyframe& keyframe : obstacle.keyframes) {
      keyframes.append(QJsonObject{{"time_s", keyframe.time_s}, {"pose", PoseToJson(keyframe.pose)},
                                   {"speed_mps", keyframe.speed_mps}});
    }
    obstacles.append(QJsonObject{{"id", QString::fromStdString(obstacle.id)}, {"length_m", obstacle.length_m},
                                 {"width_m", obstacle.width_m}, {"confidence", obstacle.confidence},
                                 {"loop", obstacle.loop}, {"keyframes", keyframes}});
  }
  root["obstacles"] = obstacles;
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
    *error = "cannot open scenario for writing";
    return false;
  }
  file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  return true;
}

bool LoadScenarioJson(const std::string& path, SimulationScenario* scenario, std::string* error) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly)) {
    *error = "cannot open scenario for reading";
    return false;
  }
  QJsonParseError parse_error;
  const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
    *error = "invalid scenario JSON";
    return false;
  }
  SimulationScenario result;
  const QJsonObject root = document.object();
  if (root["schema_version"].toInt() != 1 ||
      !EgoFromJson(root["initial_ego"].toObject(), &result.initial_ego)) {
    *error = "unsupported or incomplete scenario";
    return false;
  }
  result.target_parking_spot_id = root["target_parking_spot_id"].toString().toStdString();
  VehicleFromJson(root["vehicle"].toObject(), &result.vehicle);
  PlannerFromJson(root["planner"].toObject(), &result.planner);
  for (const QJsonValue& value : root["lanes"].toArray()) {
    const QJsonObject object = value.toObject();
    Lane lane;
    lane.id = object["id"].toString().toStdString();
    lane.closed = object["closed"].toBool();
    for (const QJsonValue& point : object["centerline"].toArray()) {
      const QJsonArray pair = point.toArray();
      if (pair.size() != 2) { *error = "invalid lane point"; return false; }
      lane.centerline.push_back({pair[0].toDouble(), pair[1].toDouble()});
    }
    for (const QJsonValue& successor : object["successor_ids"].toArray()) {
      lane.successor_ids.push_back(successor.toString().toStdString());
    }
    result.map.lanes.push_back(std::move(lane));
  }
  for (const QJsonValue& value : root["parking_spots"].toArray()) {
    const QJsonObject object = value.toObject();
    ParkingSpot spot;
    spot.id = object["id"].toString().toStdString();
    if (!PoseFromJson(object["entry_pose"].toObject(), &spot.entry_pose) ||
        !PoseFromJson(object["target_pose"].toObject(), &spot.target_pose)) {
      *error = "invalid parking spot pose";
      return false;
    }
    result.map.parking_spots.push_back(std::move(spot));
  }
  for (const QJsonValue& value : root["obstacles"].toArray()) {
    const QJsonObject object = value.toObject();
    ScenarioObstacle obstacle;
    obstacle.id = object["id"].toString().toStdString();
    obstacle.length_m = object["length_m"].toDouble();
    obstacle.width_m = object["width_m"].toDouble();
    obstacle.confidence = object["confidence"].toDouble(1.0);
    obstacle.loop = object["loop"].toBool();
    for (const QJsonValue& keyframe_value : object["keyframes"].toArray()) {
      const QJsonObject keyframe_object = keyframe_value.toObject();
      ObstacleKeyframe keyframe;
      keyframe.time_s = keyframe_object["time_s"].toDouble();
      keyframe.speed_mps = keyframe_object["speed_mps"].toDouble();
      if (!PoseFromJson(keyframe_object["pose"].toObject(), &keyframe.pose)) {
        *error = "invalid obstacle keyframe";
        return false;
      }
      obstacle.keyframes.push_back(keyframe);
    }
    if (obstacle.id.empty() || obstacle.keyframes.empty()) { *error = "incomplete obstacle"; return false; }
    std::sort(obstacle.keyframes.begin(), obstacle.keyframes.end(),
              [](const ObstacleKeyframe& left, const ObstacleKeyframe& right) {
                return left.time_s < right.time_s;
              });
    result.obstacles.push_back(std::move(obstacle));
  }
  *scenario = std::move(result);
  return true;
}

}  // namespace avp::tools
