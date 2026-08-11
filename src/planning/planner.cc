#include "planning/planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace avp {
namespace {
// 自车到泊车开始点的容差，用于判断是否已经到达泊车入口
constexpr double kParkingEntryToleranceM = 0.55;
// 车辆到泊车段投影点的偏差，如果超过这个值，则重规划
constexpr double kParkingSegmentDeviationM = 1.0;

// 查询路径在 s 处的x, y
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

// 查询路径在 s 处的 yaw
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

// 查询路径在 s 处的曲率
double CurvatureAtS(const std::vector<PathPoint>& path, double s) {
  for (size_t i = 1; i < path.size(); ++i) {
    if (s <= path[i].s) return path[i - 1].curvature;
  }
  return path.back().curvature;
}

// 计算折线总长度
double PolylineLength(const std::vector<Vec2>& points) {
  double result = 0.0;
  for (size_t index = 1; index < points.size(); ++index) {
    result += Distance(points[index - 1], points[index]);
  }
  return result;
}

// 估算车辆在一个规划时域 horizon_s 内最多能走多远
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

/**
  @brief 确定局部规划从当前位置向前截取多长的全局路线
  @note: 包含三部分：
  本规划周期可能前进的距离 + 若需要停车所需制动距离 + 车辆几何和安全缓冲
*/
double LocalPathHorizon(const PlanningFrame& frame) {
  const double braking_distance = frame.vehicle.max_speed_mps * frame.vehicle.max_speed_mps /
                                  (2.0 * frame.vehicle.max_deceleration_mps2);
  const double geometry_buffer = 0.5 * frame.vehicle.length_m + frame.vehicle.safety_margin_m;
  return MaximumTravelDistance(frame, frame.vehicle.max_speed_mps) + braking_distance +
         geometry_buffer;
}

// 表示点投影到折线后的结果
struct Projection {
  size_t segment_index = 0;  // 投影落在哪一段折线，表示段起点索引
  Vec2 position;             // 投影点坐标
  double distance = std::numeric_limits<
      double>::infinity();  // 原始点与投影点的最短距离，默认无穷大，便于后续取最小值
};

// 把车辆当前位置投影到全局参考折线上，找到最近位置
Projection ProjectToPolyline(const std::vector<Vec2>& points, const Vec2& position) {
  Projection best;
  for (size_t index = 1; index < points.size(); ++index) {
    const Vec2 delta{points[index].x - points[index - 1].x, points[index].y - points[index - 1].y};
    const double length_squared = delta.x * delta.x + delta.y * delta.y;
    const Vec2 offset{position.x - points[index - 1].x, position.y - points[index - 1].y};
    const double ratio =
        std::clamp((offset.x * delta.x + offset.y * delta.y) / length_squared, 0.0, 1.0);
    const Vec2 projected = Interpolate(points[index - 1], points[index], ratio);
    const double distance = Distance(position, projected);
    if (distance < best.distance) {
      best = {index - 1, projected, distance};
    }
  }
  return best;
}
// 从全局路线截取出来供局部规划使用的一段路线
struct CroppedRoute {
  GlobalRoute route;             // 裁剪后的路线，保留原路线的其他元信息，只替换 reference_line
  double global_length_m = 0.0;  // 原全局路线的总长度
  double local_length_m = 0.0;   // 裁剪后的局部路线长度
  bool reaches_end = false;      // 局部截取是否已经到达全局路线末端
};

// 从车辆当前位置开始，沿全局参考线向前截取 horizon_m 米
CroppedRoute CropRoute(const GlobalRoute& route, const Vec2& ego_position, double horizon_m) {
  // 复制全局路径，清空参考线
  CroppedRoute result;
  result.route = route;
  result.route.reference_line.clear();
  // 统计完整全局路线长度；将自车当前位置投影到原参考线；将投影点作为局部参考线起点。
  // 这比直接从“最近离散点”开始更连续，能减少路径跳变。
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
      // 整个线段能放入局部范围， 继续向前添加
      result.route.reference_line.push_back(end);
      remaining -= length;
      current = end;
      continue;
    }
    // 线段不能完整放入，在该段内部插一个精确终点，使局部路线长度恰好为 horizon_m
    result.route.reference_line.push_back(Interpolate(current, end, remaining / length));
    remaining = 0.0;
    break;
  }
  // 裁剪末端与全局终点重合，说明前方已经没有更多路线了。后续速度规划必须考虑终点停车
  result.reaches_end =
      Distance(result.route.reference_line.back(), route.reference_line.back()) < 1e-6;
  result.local_length_m = PolylineLength(result.route.reference_line);
  return result;
}

/**
  @brief 将局部路径规划的path加密，以便速度规划使用
  @note:
  1. 路径规划器和速度规划器对离散化的要求不一样：
      路径规划更关注几何可行性，可使用较少的层；
      速度规划需要计算速度、加速度、加加速度和动态障碍物时序，通常需要更细的 s 采样。
  2. 近处路径使用更细的采样间隔，提升控制精度和平滑性；
     远处路径使用较粗的采样间隔，降低计算量。
*/
std::vector<PathPoint> DensifyPathForSpeed(const PlanningFrame& frame,
                                           const std::vector<PathPoint>& path,
                                           double max_speed_mps) {
  std::vector<PathPoint> result;
  result.push_back(path.front());
  result.front().s = 0.0;
  // fine_step 是近处路径的采样间隔，限制在 [0.005, 0.02] m
  const double fine_step = std::max(
      0.005,
      std::min(0.02, frame.vehicle.max_jerk_mps3 * std::pow(frame.config.time_step_s, 3.0) * 0.5));
  // coarse_step 是远处路径的采样间隔，限制在 [fine_step, 0.05] m
  const double coarse_step =
      std::max(fine_step, std::min(0.05, max_speed_mps * frame.config.time_step_s * 0.25));
  const double total_length = path.back().s;
  for (double s = fine_step; s < total_length - 1e-9;) {
    // 前 0.25 m 使用更细密的采样，通常是为了提升当前车附近的控制和平滑性；
    // 之后用 coarse_step 降低计算量。
    result.push_back({PositionAtS(path, s), YawAtS(path, s), CurvatureAtS(path, s), s});
    s += s < 0.25 ? fine_step : coarse_step;
  }
  if (total_length > 1e-9) {
    result.push_back(path.back());
  }
  return result;
}

/**
  @brief 从障碍物预测轨迹中，找出不晚于指定时间的最后一个预测点
  @note
  1. 这是一种零阶保持策略：不在两个预测点间插值，而是用最近的历史预测状态近似。
  实现简单，但在高速动态障碍物场景中精度会弱于插值预测。
*/
PredictionPoint PredictAt(const Obstacle& obstacle, uint64_t timestamp_ns) {
  PredictionPoint result = obstacle.prediction.front();
  for (const PredictionPoint& point : obstacle.prediction) {
    if (point.timestamp_ns > timestamp_ns) break;
    result = point;
  }
  return result;
}

// 检查整条时间轨迹是否与任意障碍物发生碰撞
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

/**
  @brief 生成一条尽可能沿当前朝向刹停的兜底轨迹
  @note
  注意：
  1. 紧急轨迹可能仍然与障碍物冲突，因此调用方会额外记录 stop_collision_free 诊断信息。
     这是“无安全规划结果时的最后输出”，不代表绝对安全。
*/
std::vector<TimedTrajectoryPoint> MakeStopTrajectory(const PlanningFrame& frame) {
  std::vector<TimedTrajectoryPoint> result;
  const DrivingDirection direction = frame.ego.direction == DrivingDirection::kReverse
                                         ? DrivingDirection::kReverse
                                         : DrivingDirection::kDrive;
  const double direction_sign = direction == DrivingDirection::kReverse ? -1.0 : 1.0;
  const double initial_speed = std::abs(frame.ego.speed_mps);
  const double deceleration = std::max(0.1, frame.vehicle.max_deceleration_mps2);
  // 至少保留一个规划周期的轨迹点
  const double stop_time = std::max(frame.config.time_step_s, initial_speed / deceleration);
  for (double t = 0.0; t <= stop_time + 1e-9; t += frame.config.time_step_s) {
    const double speed = std::max(0.0, initial_speed - deceleration * t);
    const double distance = std::max(0.0, initial_speed * t - 0.5 * deceleration * t * t);
    // 把沿车体纵向的行驶距离转换到世界坐标
    const Vec2 position{
        frame.ego.pose.position.x + direction_sign * std::cos(frame.ego.pose.yaw) * distance,
        frame.ego.pose.position.y + direction_sign * std::sin(frame.ego.pose.yaw) * distance};
    result.push_back({{position, frame.ego.pose.yaw}, 0.0, speed, -deceleration, t, direction});
  }
  return result;
}

/**
  @brief 生成整段时域内完全静止的轨迹，主要用于停车和换挡
  @note
*/
std::vector<TimedTrajectoryPoint> MakeStationaryTrajectory(const PlanningFrame& frame,
                                                           DrivingDirection direction) {
  std::vector<TimedTrajectoryPoint> result;
  const int count =
      static_cast<int>(std::floor(frame.config.horizon_s / frame.config.time_step_s)) + 1;
  result.reserve(count);
  // 位姿、曲率、速度、加速度都不变，只有时间递增
  for (int index = 0; index < count; ++index) {
    result.push_back({frame.ego.pose, 0.0, 0.0, 0.0, index * frame.config.time_step_s, direction});
  }
  return result;
}

// 统一构造规划失败后的响应
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

/**
  @brief 从一个 Hybrid A* 泊车段中，截取从当前车辆位置开始、在本规划时域内需要执行的一小段路径
  @param frame 当前规划帧
  @param segment 当前泊车段
  @param max_length_m 本周期最多需要准备的路径长度
  @param max_speed_mps 该段的速度上限
  @param nearest_distance_m 输出车辆当前位置到该段的最近点距离
  @param reaches_end 输出截取路径是否抵达当前泊车段终点

*/
std::vector<PathPoint> BuildParkingPath(const PlanningFrame& frame, const ParkingSegment& segment,
                                        double max_length_m, double max_speed_mps,
                                        double* nearest_distance_m, bool* reaches_end) {
  // 找最近点的index和距离
  size_t nearest = 0;
  *nearest_distance_m = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < segment.points.size(); ++index) {
    const double distance = Distance(frame.ego.pose.position, segment.points[index].pose.position);
    if (distance < *nearest_distance_m) {
      *nearest_distance_m = distance;
      nearest = index;
    }
  }
  // 局部路径起点强制使用当前自车真实位姿，而不是直接使用 Hybrid A* 的最近点
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
    coarse.push_back({point.pose.position, point.pose.yaw, point.signed_curvature_1pm, s});
  }
  *reaches_end = true;
  return DensifyPathForSpeed(frame, coarse, max_speed_mps);
}

// 把几何路径和速度曲线合成为最终时域轨迹
std::vector<TimedTrajectoryPoint> ComposeTrajectory(const std::vector<PathPoint>& path,
                                                    const std::vector<SpeedPoint>& speed,
                                                    DrivingDirection direction) {
  std::vector<TimedTrajectoryPoint> result;
  result.reserve(speed.size());
  for (const SpeedPoint& point : speed) {
    result.push_back({{PositionAtS(path, point.s), YawAtS(path, point.s)},
                      CurvatureAtS(path, point.s),
                      point.speed_mps,
                      point.acceleration_mps2,
                      point.time_s,
                      direction});
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

// 当目标车位切换时，重置所有与旧任务有关的状态。
void Planner::ResetTask(const std::string& target_parking_spot_id) {
  active_target_parking_spot_id_ = target_parking_spot_id;
  mode_ = Mode::kLaneApproach;
  parking_maneuver_.segments.clear();
  parking_segment_index_ = 0;
  gear_shift_start_timestamp_ns_ = 0;
  last_parking_replan_timestamp_ns_ = 0;
  pending_direction_ = DrivingDirection::kUnknown;
}

bool Planner::ReplanParking(const PlanningFrame& frame, std::string* error) {
  if (last_parking_replan_timestamp_ns_ == frame.header.timestamp_ns) {
    *error = "parking replan already attempted for current frame";
    return false;
  }

  last_parking_replan_timestamp_ns_ = frame.header.timestamp_ns;
  const ParkingSpot* spot = nullptr;
  for (const ParkingSpot& candidate : frame.map->parking_spots) {
    if (candidate.id == frame.target_parking_spot_id) {
      spot = &candidate;
      break;
    }
  }
  if (spot == nullptr) {
    *error = "target parking spot not found during parking replan";
    return false;
  }

  ParkingManeuver replanned;
  if (!hybrid_a_star_.Plan(frame, frame.ego.pose, spot->target_pose, &replanned, error)) {
    return false;
  }

  parking_maneuver_ = std::move(replanned);
  parking_segment_index_ = 0;
  mode_ = Mode::kOpenSpaceParking;
  gear_shift_start_timestamp_ns_ = 0;
  pending_direction_ = DrivingDirection::kUnknown;
  return true;
}

PlanningResponse Planner::PlanParking(const PlanningFrame& frame) {
  while (true) {
    // 1. 判断全部泊车段是否执行完毕
    if (parking_segment_index_ >= parking_maneuver_.segments.size()) {
      PlanningResponse response;
      response.header = frame.header;
      response.status = PlanningStatus::kOk;
      response.message = "parking maneuver completed";
      response.trajectory = MakeStationaryTrajectory(frame, frame.ego.direction);
      response.diagnostics = {"planning_mode=OPEN_SPACE_PARKING", "parking_complete=true"};
      return response;
    }

    // 2. 换挡逻辑
    if (mode_ == Mode::kGearShift) {
      const double elapsed_s =
          frame.header.timestamp_ns >= gear_shift_start_timestamp_ns_
              ? static_cast<double>(frame.header.timestamp_ns - gear_shift_start_timestamp_ns_) /
                    1e9
              : 0.0;
      if (std::abs(frame.ego.speed_mps) > frame.config.gear_shift_stop_speed_mps ||
          elapsed_s < frame.config.gear_shift_dwell_s ||
          frame.ego.direction != pending_direction_) {
        // 若满足以上任一条件，则继续等待换挡完成，输出静止轨迹
        // 车速还没低到换挡阈值 || 换挡等待时间还没到 ||
        // 底盘/控制反馈的当前挡位还没有切换为目标方向
        PlanningResponse response;
        response.header = frame.header;
        response.status = PlanningStatus::kOk;
        response.message = "waiting for safe gear shift";
        response.trajectory = MakeStationaryTrajectory(frame, pending_direction_);
        response.diagnostics = {"planning_mode=GEAR_SHIFT",
                                std::string("gear=") + ToString(pending_direction_),
                                "parking_segment_index=" + std::to_string(parking_segment_index_)};
        return response;
      }
      mode_ = Mode::kOpenSpaceParking;
    }

    // 每次循环都在状态更新后重新取得当前段，避免引用已被替换的泊车动作。
    const ParkingSegment& segment = parking_maneuver_.segments[parking_segment_index_];

    // 3. 档位不一致时，进入换挡阶段
    if (frame.ego.direction != segment.direction) {
      mode_ = Mode::kGearShift;
      pending_direction_ = segment.direction;
      gear_shift_start_timestamp_ns_ = frame.header.timestamp_ns;
      continue;
    }

    // 4. 生成当前泊车段局部路径
    const double max_speed = segment.direction == DrivingDirection::kReverse
                                 ? frame.config.max_reverse_speed_mps
                                 : frame.vehicle.max_speed_mps;
    const double max_path_length = MaximumTravelDistance(frame, max_speed);
    double nearest_distance = 0.0;
    bool reaches_end = false;
    std::vector<PathPoint> path = BuildParkingPath(frame, segment, max_path_length, max_speed,
                                                   &nearest_distance, &reaches_end);
    if (nearest_distance > kParkingSegmentDeviationM) {
      // 偏离旧泊车轨迹时，同一规划帧最多重规划一次。
      std::string replan_error;
      if (ReplanParking(frame, &replan_error)) {
        continue;
      }
      return MakeFallbackResponse(frame,
                                  replan_error.empty() ? "parking replan failed" : replan_error,
                                  "OPEN_SPACE_PARKING");
    }

    // 5. 判断当前泊车段是否结束（点数不够 ||
    // 当前局部路径覆盖段终点，车辆位置接近段终点，并且已经基本停住）
    if (path.size() < 2 ||
        (reaches_end &&
         Distance(frame.ego.pose.position, segment.points.back().pose.position) <
             kParkingEntryToleranceM &&
         std::abs(frame.ego.speed_mps) <= frame.config.gear_shift_stop_speed_mps)) {
      ++parking_segment_index_;
      if (parking_segment_index_ < parking_maneuver_.segments.size()) {
        pending_direction_ = parking_maneuver_.segments[parking_segment_index_].direction;
        mode_ = Mode::kGearShift;
        gear_shift_start_timestamp_ns_ = frame.header.timestamp_ns;
      }
      continue;
    }

    // 6. 为泊车路径生成速度
    std::vector<SpeedPoint> speed;
    std::string error;
    if (!speed_planner_.Plan(frame, path, &speed, &error, {max_speed, false})) {
      return MakeFallbackResponse(frame, error, "OPEN_SPACE_PARKING");
    }
    if (reaches_end && speed.back().s >= path.back().s - 0.1) {
      // 若当前局部路径已覆盖泊车段终点 && 正常速度规划也确实会走到路径末端附近
      // 重新规划 => 要求在终点停车
      std::vector<SpeedPoint> stopped_speed;
      std::string stop_error;
      if (speed_planner_.Plan(frame, path, &stopped_speed, &stop_error, {max_speed, true})) {
        speed = std::move(stopped_speed);
      }
    }

    // 7. 轨迹合成、碰撞校验和二次重规划
    PlanningResponse response;
    response.header = frame.header;
    response.trajectory = ComposeTrajectory(path, speed, segment.direction);
    if (!IsCollisionFree(frame, response.trajectory)) {
      std::string replan_error;
      if (ReplanParking(frame, &replan_error)) {
        continue;
      }
      return MakeFallbackResponse(frame, "post-plan parking collision validation failed",
                                  "OPEN_SPACE_PARKING");
    }

    // 规划成功时，返回response
    response.status = PlanningStatus::kOk;
    response.message = "open-space parking trajectory generated";
    response.diagnostics = {
        "planning_mode=OPEN_SPACE_PARKING", std::string("gear=") + ToString(segment.direction),
        "parking_segment_index=" + std::to_string(parking_segment_index_), "speed=ST_DP"};
    return response;
  }
}

// 整个规划的入口函数
PlanningResponse Planner::Plan(const PlanningRequest& request) {
  // 先保留请求头，便于下游匹配时间戳和请求序列
  PlanningResponse response;
  response.header = request.header;
  // 1. 将请求数据适配为规划帧, 并检查输入合法性
  PlanningFrame frame;
  std::string error;
  if (!adapter_.Adapt(request, vehicle_, config_, &frame, &error)) {
    response.status = PlanningStatus::kInvalidInput;
    response.message = error;
    return response;
  }
  // 2. 检查目标车位是否切换
  if (active_target_parking_spot_id_ != frame.target_parking_spot_id) {
    // 目标改变时，清空旧任务的泊车路径、段索引、换挡等待状态
    ResetTask(frame.target_parking_spot_id);
  }
  // 3. 一旦进入泊车流程，不再每周期重新生成全局路线，而是直接执行当前 Hybrid A* 泊车动作
  // 这保证泊车阶段的行为连续，避免因全局路线变化造成状态混乱
  if (mode_ != Mode::kLaneApproach) {
    return PlanParking(frame);
  }
  // 4. 生成全局路线
  GlobalRoute route;
  if (!global_planner_.Plan(frame, &route, &error)) {
    response.status = PlanningStatus::kNoRoute;
    response.message = error;
    return response;
  }
  // 5. 判断是否进入泊车模式
  if (Distance(frame.ego.pose.position, route.parking_entry.position) <= kParkingEntryToleranceM &&
      std::abs(frame.ego.speed_mps) <= frame.config.gear_shift_stop_speed_mps) {
    if (Distance(frame.ego.pose.position, route.parking_target.position) <=
            kParkingEntryToleranceM &&
        std::abs(NormalizeAngle(frame.ego.pose.yaw - route.parking_target.yaw)) < 0.45) {
      // 这表示已经基本泊好，不必再调用 Hybrid A*
      // 空的 segments 会让 PlanParking() 返回“parking maneuver completed”的静止轨迹
      parking_segment_index_ = 0;
      parking_maneuver_.segments.clear();
      return PlanParking(frame);
    }
    if (!hybrid_a_star_.Plan(frame, frame.ego.pose, route.parking_target, &parking_maneuver_,
                             &error)) {
      return MakeFallbackResponse(frame, error, "OPEN_SPACE_PARKING");
    }
    parking_segment_index_ = 0;
    mode_ = Mode::kOpenSpaceParking;
    return PlanParking(frame);
  }
  // 6. 从全局路径截取部分路径，进行局部路径规划
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
  // 7. 路径—速度耦合迭代
  std::vector<double> arrivals;       // 局部路径各离散点的预计到达时刻
  std::vector<PathPoint> path;        // 局部规划器输出的原始路径
  std::vector<PathPoint> speed_path;  // 局部规划器输出的加密路径，用于速度规划
  std::vector<SpeedPoint> speed;      // 速度规划器输出的速度曲线
  // note
  // 进行固定次数的s-l s-t耦合迭代
  for (int iteration = 0; iteration < frame.config.path_coupling_iterations; ++iteration) {
    std::vector<PathPoint> candidate_path;
    std::vector<PathPoint> candidate_speed_path;
    std::vector<SpeedPoint> candidate_speed;
    // 第一轮 arrivals为空，后续轮次由上轮速度规划结果计算得到
    if (!local_planner_.Plan(frame, cropped.route, arrivals, &candidate_path, &error)) {
      break;
    }
    candidate_speed_path = DensifyPathForSpeed(frame, candidate_path, frame.vehicle.max_speed_mps);
    if (!speed_planner_.Plan(frame, candidate_speed_path, &candidate_speed, &error,
                             {frame.vehicle.max_speed_mps, false})) {
      break;
    }
    if (cropped.reaches_end && candidate_speed.back().s >= candidate_speed_path.back().s - 0.1) {
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

    // 更新到达时间，供路径规划使用
    arrivals.assign(path.size(), frame.config.horizon_s);
    for (size_t index = 0; index < path.size(); ++index) {
      for (const auto& speed_point : speed) {
        if (speed_point.s + 1e-6 >= path[index].s) {
          arrivals[index] = speed_point.time_s;
          break;
        }
      }
    }
  }
  // 8. 输出最终轨迹与安全校验
  if (path.empty() || speed_path.empty() || speed.empty()) {
    // 路径为空，就走紧急停车轨迹
    return MakeFallbackResponse(
        frame, error.empty() ? "no feasible local path or speed profile" : error, "LANE_APPROACH");
  }

  response.trajectory = ComposeTrajectory(speed_path, speed, DrivingDirection::kDrive);
  if (!IsCollisionFree(frame, response.trajectory)) {
    // 做独立的后验碰撞检查；失败后返回紧急停车
    return MakeFallbackResponse(frame, "post-plan collision validation failed", "LANE_APPROACH");
  }

  response.status = PlanningStatus::kOk;
  response.message = "rolling-horizon spatiotemporal DP trajectory generated";
  response.diagnostics = {"planning_mode=LANE_APPROACH",
                          "global=A_STAR",
                          "local=SL_DP",
                          "speed=ST_DP",
                          "gear=DRIVE",
                          Metric("global_route_length_m", cropped.global_length_m),
                          Metric("local_horizon_m", cropped.local_length_m),
                          "path_layer_count=" + std::to_string(path.size())};
  return response;
}
}  // namespace avp
