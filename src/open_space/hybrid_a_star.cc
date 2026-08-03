#include "open_space/hybrid_a_star.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <sstream>
#include <unordered_map>

namespace avp {
namespace {
// 搜索节点
struct Node {
  Pose2d pose;      // 当前节点的位姿
  int parent = -1;  // 父节点在nodes数组中的下标，-1表示没有父节点
  double g = 0.0;   // 当前节点的历史代价
  double signed_curvature_1pm = 0.0;
  DrivingDirection direction = DrivingDirection::kUnknown;
};
// 优先队列里的元素
struct Entry {
  double f;   // 估计总代价 f = g + h
  int index;  // 节点在nodes数组中的下标
  bool operator>(const Entry& other) const { return f > other.f; }
};
// 把连续状态离散化，生成唯一的键值，用于哈希表。
std::string Key(const Pose2d& pose, DrivingDirection direction) {
  std::ostringstream stream;
  stream << std::llround(pose.position.x * 2.0) << ':' << std::llround(pose.position.y * 2.0) << ':'
         << std::llround(NormalizeAngle(pose.yaw) * 12.0 / kPi) << ':'
         << static_cast<int>(direction);
  return stream.str();
}
// 粗略的启发函数，粗略引导
double Heuristic(const Pose2d& pose, const Pose2d& goal) {
  return Distance(pose.position, goal.position) +
         0.5 * std::abs(NormalizeAngle(goal.yaw - pose.yaw));
}
// 碰撞检测，使用分离轴定理检测两个有向矩形。自车半尺寸按安全边距膨胀，边界接触视为碰撞。
bool Collides(const PlanningFrame& frame, const Pose2d& pose) {
  for (const Obstacle& obstacle : frame.obstacles) {
    const PredictionPoint& prediction = obstacle.prediction.front();
    if (IsVehicleObstacleCollision(pose, frame.vehicle, prediction.pose, obstacle.length_m,
                                   obstacle.width_m)) {
      return true;
    }
  }
  return false;
}
}  // namespace

bool HybridAStar::Plan(const PlanningFrame& frame, const Pose2d& start, const Pose2d& goal,
                       ParkingManeuver* maneuver, std::string* error) const {
  if (maneuver == nullptr || error == nullptr) {
    return false;
  }
  // 1. 搜索参数定义
  constexpr double kStepM = 0.5;                       // 每次扩展走0.5m
  constexpr int kMaxExpansions = 30000;                // 最大扩展节点数，防止搜索过久
  const double curvature_scales[] = {-0.8, 0.0, 0.8};  // 三种转向动作，左转-直行-右转
  // 2. 初始化 open list
  std::vector<Node> nodes;
  nodes.reserve(8192);
  nodes.push_back({start, -1, 0.0, 0.0, DrivingDirection::kUnknown});
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  std::unordered_map<std::string, double> best_cost{
      {Key(start, DrivingDirection::kUnknown), 0.0}};
  open.push({Heuristic(start, goal), 0});
  int goal_index = -1;
  // 3. A* 主循环
  for (int expansion = 0; !open.empty() && expansion < kMaxExpansions; ++expansion) {
    // 取出代价最小的节点
    const int index = open.top().index;
    open.pop();
    const Node current = nodes[index];
    // 判断是否到达目标
    if (Distance(current.pose.position, goal.position) < 0.55 &&
        std::abs(NormalizeAngle(current.pose.yaw - goal.yaw)) < 0.45) {
      goal_index = index;
      break;
    }
    // 扩展动作：前进/倒车 + 左/直/右
    for (const double direction : {1.0, -1.0}) {
      for (const double scale : curvature_scales) {
        // 生成 next 位姿
        const double curvature = scale * frame.vehicle.max_curvature_1pm;
        const DrivingDirection next_direction =
            direction > 0.0 ? DrivingDirection::kDrive : DrivingDirection::kReverse;
        Pose2d next = current.pose;
        next.yaw = NormalizeAngle(next.yaw + direction * kStepM * curvature);
        next.position.x += direction * kStepM * std::cos(next.yaw);
        next.position.y += direction * kStepM * std::sin(next.yaw);
        // 碰撞过滤
        if (Collides(frame, next)) {
          continue;
        }
        // 计算历史代价 g = 当前代价 + 行驶距离 + 倒车惩罚 + 转弯惩罚
        const double gear_switch_cost =
            current.direction != DrivingDirection::kUnknown &&
                    current.direction != next_direction
                ? 2.0
                : 0.0;
        const double g = current.g + kStepM + (direction < 0.0 ? 0.2 : 0.0) +
                         std::abs(curvature) * 0.05 + gear_switch_cost;
        // best_cost 剪枝
        const std::string key = Key(next, next_direction);
        if (best_cost.contains(key) && best_cost[key] <= g) {
          // 如果已经有更优的代价，则跳过当前节点
          continue;
        }
        // 加入新节点
        best_cost[key] = g;
        nodes.push_back({next, index, g, curvature, next_direction});
        open.push({g + Heuristic(next, goal), static_cast<int>(nodes.size()) - 1});
      }
    }
  }
  // 搜索失败
  if (goal_index < 0) {
    *error = "hybrid A* search failed to find a path";
    return false;
  }
  // 4. 回溯路径
  std::vector<HybridPathPoint> connection;
  for (int index = goal_index; index >= 0; index = nodes[index].parent) {
    const Node& node = nodes[index];
    connection.push_back({node.pose, node.signed_curvature_1pm, node.direction});
  }
  std::reverse(connection.begin(), connection.end());
  if (connection.size() < 2) {
    *error = "hybrid A* connection has no usable motion";
    return false;
  }
  connection.front().direction = connection[1].direction;
  connection.front().signed_curvature_1pm = connection[1].signed_curvature_1pm;
  // 补上精确目标姿态，同时继承最后一个搜索动作的档位。
  if (Distance(connection.back().pose.position, goal.position) > 1e-6 ||
      std::abs(NormalizeAngle(connection.back().pose.yaw - goal.yaw)) > 1e-6) {
    connection.push_back({goal, connection.back().signed_curvature_1pm,
                          connection.back().direction});
  } else {
    connection.back().pose = goal;
  }

  maneuver->segments.clear();
  for (size_t index = 0; index < connection.size(); ++index) {
    const HybridPathPoint& point = connection[index];
    if (maneuver->segments.empty() ||
        maneuver->segments.back().direction != point.direction) {
      if (!maneuver->segments.empty()) {
        // 方向切换点同时作为上一段终点和下一段起点。
        HybridPathPoint cusp = connection[index - 1];
        cusp.direction = point.direction;
        cusp.signed_curvature_1pm = point.signed_curvature_1pm;
        maneuver->segments.push_back({point.direction, {cusp, point}});
      } else {
        maneuver->segments.push_back({point.direction, {point}});
      }
    } else {
      maneuver->segments.back().points.push_back(point);
    }
  }
  for (const ParkingSegment& segment : maneuver->segments) {
    if (segment.points.size() < 2) {
      *error = "hybrid A* produced a degenerate direction segment";
      maneuver->segments.clear();
      return false;
    }
  }
  return true;
}
}  // namespace avp
