#include "map/global_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

namespace avp {
namespace {
constexpr double kGeometryEpsilon = 1e-6;
constexpr double kAppendEpsilon = 1e-4;

struct LaneProjection {
  const Lane* lane = nullptr;                                 // 所属车道。
  size_t segment_index = 0;                                   // 投影所在的线段索引。
  double segment_ratio = 0.0;                                 // 投影在线段内的比例。
  Vec2 position;                                              // 投影点坐标。
  double s = 0.0;                                             // 沿车道的累计距离。
  double yaw = 0.0;                                           // 投影处车道朝向。
  double distance = std::numeric_limits<double>::infinity();  // 到查询点的距离。
};

// 计算车道中心线长度。
double LaneLength(const Lane& lane) {
  double result = 0.0;
  for (size_t i = 1; i < lane.centerline.size(); ++i) {
    result += Distance(lane.centerline[i - 1], lane.centerline[i]);
  }
  return result;
}

// 构造车道起点的投影信息。
LaneProjection LaneStart(const Lane& lane) {
  const Vec2& first = lane.centerline.front();  // 首个中心线点。
  const Vec2& second = lane.centerline[1];      // 第二个中心线点。
  return {&lane, 0, 0.0, first, 0.0, std::atan2(second.y - first.y, second.x - first.x), 0.0};
}

// 构造车道终点的投影信息。
LaneProjection LaneEnd(const Lane& lane) {
  const size_t segment_index = lane.centerline.size() - 2;  // 末段中心线索引。
  const Vec2& first = lane.centerline[segment_index];       // 末段起点。
  const Vec2& last = lane.centerline.back();                // 车道终点。
  return {&lane, segment_index,    1.0,
          last,  LaneLength(lane), std::atan2(last.y - first.y, last.x - first.x),
          0.0};
}

// 把一个二维点 position 投影到车道中心线 lane.centerline 上，找到最近点，并返回投影结果。
std::optional<LaneProjection> ProjectOntoLane(const Lane& lane, const Vec2& position) {
  std::optional<LaneProjection> best;  // 当前最近投影。
  double s_before_segment = 0.0;       // 当前线段前的累计距离。
  for (size_t index = 1; index < lane.centerline.size(); ++index) {
    const Vec2& start = lane.centerline[index - 1];
    const Vec2& end = lane.centerline[index];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < kGeometryEpsilon * kGeometryEpsilon) {
      continue;
    }
    const double segment_length = std::sqrt(squared_length);
    const double unclamped_ratio = ((position.x - start.x) * dx + (position.y - start.y) * dy) /
                                   squared_length;                      // 原始投影比例。
    const double ratio = std::clamp(unclamped_ratio, 0.0, 1.0);         // 截断后的投影比例。
    const Vec2 projection{start.x + ratio * dx, start.y + ratio * dy};  // 线段投影点。
    const double distance = Distance(position, projection);             // 到投影点的距离。
    if (!best.has_value() || distance < best->distance) {
      best = LaneProjection{&lane,
                            index - 1,
                            ratio,
                            projection,
                            s_before_segment + ratio * segment_length,
                            std::atan2(dy, dx),
                            distance};
    }
    s_before_segment += segment_length;
  }
  return best;
}

// 在所有未关闭的车道中，寻找查询点 position
// 的最佳车道投影。这个投影不仅要距离足够近，还可以选择性地满足朝向约束。
std::optional<LaneProjection> ClosestProjection(const std::vector<Lane>& lanes,
                                                const Vec2& position, const PlannerConfig& config,
                                                std::optional<double> required_yaw) {
  std::optional<LaneProjection> best;
  for (const Lane& lane : lanes) {
    if (lane.closed) {
      continue;
    }
    const std::optional<LaneProjection> candidate =
        ProjectOntoLane(lane, position);  // 当前车道投影。
    // 如果投影不存在，或者距离超过最大匹配距离，则跳过。
    if (!candidate.has_value() || candidate->distance > config.max_lane_match_distance_m) {
      continue;
    }
    // 如果要求朝向，并且当前投影的车道朝向与要求的朝向差异过大，则跳过。
    if (required_yaw.has_value() && std::abs(NormalizeAngle(*required_yaw - candidate->yaw)) >
                                        config.max_lane_heading_difference_rad) {
      continue;
    }
    if (!best.has_value() || candidate->distance < best->distance) {
      best = candidate;
    }
  }
  return best;
}

// 向参考线追加不重复的点。
void AppendPoint(std::vector<Vec2>* reference_line, const Vec2& point) {
  if (reference_line->empty() || Distance(reference_line->back(), point) > kAppendEpsilon) {
    reference_line->push_back(point);
  }
}

// 把同一条车道上，从 start 投影位置到 end 投影位置之间的中心线路径，依次追加到 reference_line 中。
void AppendLaneSlice(std::vector<Vec2>* reference_line, const LaneProjection& start,
                     const LaneProjection& end) {
  AppendPoint(reference_line, start.position);
  for (size_t index = start.segment_index + 1; index <= end.segment_index; ++index) {
    AppendPoint(reference_line, start.lane->centerline[index]);
  }
  AppendPoint(reference_line, end.position);
}

}  // namespace

// 规划从自车位置到目标车位的全局参考线。
bool GlobalPlanner::Plan(const PlanningFrame& frame, GlobalRoute* route, std::string* error) const {
  if (route == nullptr || error == nullptr || frame.map == nullptr) {
    return false;
  }
  const std::vector<Lane>& lanes = frame.map->lanes;  // 地图中的全部车道。

  // 1. 确定目标车位
  const ParkingSpot* spot = nullptr;
  for (const ParkingSpot& candidate : frame.map->parking_spots) {
    if (candidate.id == frame.target_parking_spot_id) {
      spot = &candidate;
      break;
    }
  }
  if (spot == nullptr) {
    *error = "target parking spot not found";
    return false;
  }

  // 2. 寻找自车位置的最佳车道投影和停车位入口的最佳车道投影。
  const std::optional<LaneProjection> start =
      ClosestProjection(lanes, frame.ego.pose.position, frame.config, frame.ego.pose.yaw);
  if (!start.has_value()) {
    *error = "no open lane matches ego position and heading";
    return false;
  }
  const std::optional<LaneProjection> goal =
      ClosestProjection(lanes, spot->entry_pose.position, frame.config, std::nullopt);
  if (!goal.has_value()) {
    *error = "no open lane near parking entry";
    return false;
  }

  // 3. 建立车道id到车道指针的映射，便于快速查找。
  std::unordered_map<std::string, const Lane*> lane_by_id;
  for (const Lane& lane : lanes) {
    lane_by_id[lane.id] = &lane;
  }

  route->lane_ids.clear();
  // 3.1 起点和终点同车道的处理
  if (start->lane->id == goal->lane->id) {
    if (goal->s + kGeometryEpsilon < start->s) {
      *error = "parking entry is behind ego on the same directed lane";
      return false;
    }
    route->lane_ids.push_back(start->lane->id);
  }
  // 3.2 起点和终点不同车道的处理（使用 A* 搜索车道图）
  else {
    // A*初始化
    // Heuristic 函数：当前车道到目标车道的启发式代价，使用欧几里得距离。
    const auto heuristic = [&lane_by_id, &goal](const std::string& lane_id) {
      if (lane_id == goal->lane->id) {
        return 0.0;
      }
      return Distance(lane_by_id.at(lane_id)->centerline.back(), goal->position);
    };
    // A* 队列元素
    struct QueueEntry {
      double priority = 0.0;  // 总代价
      double g_cost = 0.0;    // 历史代价
      std::string id;         // 当前车道 ID
      // 按优先级比较队列元素。
      bool operator>(const QueueEntry& other) const { return priority > other.priority; }
    };
    const double start_cost = LaneLength(*start->lane) - start->s;  // 起点至首车道终点的代价。
    // A* 待扩展队列。
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    // 历史代价hash表，记录到达每个车道的最小代价。
    std::unordered_map<std::string, double> g_cost{{start->lane->id, start_cost}};
    // 最优路径hash表，用来回溯从起点到终点的车道路径<当前车道id, 父节点车道id>
    std::unordered_map<std::string, std::string> parent;
    open.push({start_cost + heuristic(start->lane->id), start_cost, start->lane->id});
    // A* 主循环， 直到找到目标车道或队列为空。
    while (!open.empty()) {
      const QueueEntry current = open.top();  // 返回队首元素
      open.pop();                             // 删除队首元素
      if (current.g_cost > g_cost.at(current.id)) {
        // 跳过过时的队列元素，当前车道已经有更优的代价。
        continue;
      }
      if (current.id == goal->lane->id) {
        // 到达目标车道，结束搜索。
        break;
      }
      const Lane* current_lane = lane_by_id.at(current.id);               
      for (const std::string& successor : current_lane->successor_ids) {  // 遍历后继车道。
        const Lane* successor_lane = lane_by_id.at(successor);    
        if (successor_lane->closed) {
          continue;
        }
        const double traversed_length = successor == goal->lane->id
                                            ? goal->s
                                            : LaneLength(*successor_lane);  
        const double next_cost = current.g_cost + traversed_length;  
        // 如果找到更优的代价，则更新历史代价和父节点，并将后继车道加入队列。
        if (!g_cost.contains(successor) || next_cost < g_cost[successor]) {
          g_cost[successor] = next_cost;
          parent[successor] = current.id;
          open.push({next_cost + heuristic(successor), next_cost, successor});
        }
      }
    }
    // 判断目标是否可达
    if (!g_cost.contains(goal->lane->id)) {
      *error = "parking entry is unreachable in lane graph";
      return false;
    }
    // 4. 回溯从目标车道到起点车道的路径，构建车道 ID 序列。
    std::vector<std::string> reversed;  
    for (std::string lane_id = goal->lane->id;; lane_id = parent.at(lane_id)) {  // 回溯前驱车道。
      reversed.push_back(lane_id);
      if (lane_id == start->lane->id) {
        break;
      }
    }
    // 4.1 反转路径，并写入结果
    route->lane_ids.assign(reversed.rbegin(), reversed.rend());
  }

  // 5 根据车道 ID 序列，拼接各车道中心线，构建全局参考线。
  route->reference_line.clear();
  for (size_t index = 0; index < route->lane_ids.size(); ++index) {  
    const Lane* lane = lane_by_id.at(route->lane_ids[index]);        
    if (index == 0 && index + 1 == route->lane_ids.size()) {
      // 起点和终点在同一车道，直接从起点到终点。
      AppendLaneSlice(&route->reference_line, *start, *goal);
    } else if (index == 0) {
      // 起点车道，追加从起点到车道终点的中心线。
      AppendLaneSlice(&route->reference_line, *start, LaneEnd(*lane));
    } else if (index + 1 == route->lane_ids.size()) {
      // 终点车道，追加从车道起点到终点的中心线。
      AppendLaneSlice(&route->reference_line, LaneStart(*lane), *goal);
    } else {
      // 中间车道，追加整条车道的中心线。
      AppendLaneSlice(&route->reference_line, LaneStart(*lane), LaneEnd(*lane));
    }
  }
  // 追加停车位入口原始坐标，因为停车入口在车道中心线上的最近投影点和停车入口原始坐标可能有偏差，直接使用原始坐标可以保证停车入口的精确性。
  AppendPoint(&route->reference_line, spot->entry_pose.position);

  // 6. Hybrid A* 连接停车入口与目标位姿
  std::vector<Pose2d> connection;  // 车位入口至停车目标的连接轨迹。
  if (!hybrid_a_star_.Plan(frame, spot->entry_pose, spot->target_pose, &connection, error)) {
    return false;
  }
  // 把 Hybrid A* 轨迹追加到参考线
  for (const Pose2d& point : connection) {
    AppendPoint(&route->reference_line, point.position);
  }
  // 最后校验参考线长度是否足够
  if (route->reference_line.size() < 2) {
    *error = "reference line has no usable length";
    return false;
  }
  // 保存停车目标， 包含位置和朝向
  route->parking_target = spot->target_pose;
  return true;
}
}  // namespace avp
