#include "open_space/hybrid_a_star.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <sstream>
#include <unordered_map>

namespace avp {
namespace {
struct Node { Pose2d pose; int parent = -1; double g = 0.0; };
struct Entry { double f; int index; bool operator>(const Entry& other) const { return f > other.f; } };
std::string Key(const Pose2d& pose) {
  std::ostringstream stream;
  stream << std::llround(pose.position.x * 2.0) << ':' << std::llround(pose.position.y * 2.0) << ':'
         << std::llround(NormalizeAngle(pose.yaw) * 12.0 / kPi);
  return stream.str();
}
double Heuristic(const Pose2d& pose, const Pose2d& goal) {
  return Distance(pose.position, goal.position) + 0.5 * std::abs(NormalizeAngle(goal.yaw - pose.yaw));
}
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
}  // 匿名命名空间

bool HybridAStar::Plan(const PlanningFrame& frame, const Pose2d& start, const Pose2d& goal,
                       std::vector<Pose2d>* connection, std::string* error) const {
  if (connection == nullptr || error == nullptr) return false;
  constexpr double kStepM = 0.5;
  constexpr int kMaxExpansions = 30000;
  const double curvature_scales[] = {-0.8, 0.0, 0.8};
  std::vector<Node> nodes{{start, -1, 0.0}};
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  std::unordered_map<std::string, double> best_cost{{Key(start), 0.0}};
  open.push({Heuristic(start, goal), 0});
  int goal_index = -1;
  for (int expansion = 0; !open.empty() && expansion < kMaxExpansions; ++expansion) {
    const int index = open.top().index;
    open.pop();
    const Node current = nodes[index];
    if (Distance(current.pose.position, goal.position) < 0.55 &&
        std::abs(NormalizeAngle(current.pose.yaw - goal.yaw)) < 0.45) {
      goal_index = index;
      break;
    }
    for (const double direction : {1.0, -1.0}) {
      for (const double scale : curvature_scales) {
        const double curvature = scale * frame.vehicle.max_curvature_1pm;
        Pose2d next = current.pose;
        next.yaw = NormalizeAngle(next.yaw + direction * kStepM * curvature);
        next.position.x += direction * kStepM * std::cos(next.yaw);
        next.position.y += direction * kStepM * std::sin(next.yaw);
        if (Collides(frame, next)) continue;
        const double g = current.g + kStepM + (direction < 0.0 ? 0.2 : 0.0) +
                         std::abs(curvature) * 0.05;
        const std::string key = Key(next);
        if (best_cost.contains(key) && best_cost[key] <= g) continue;
        best_cost[key] = g;
        nodes.push_back({next, index, g});
        open.push({g + Heuristic(next, goal), static_cast<int>(nodes.size()) - 1});
      }
    }
  }
  if (goal_index < 0) {
    *error = "hybrid A* could not connect parking entry to target pose";
    return false;
  }
  connection->clear();
  for (int index = goal_index; index >= 0; index = nodes[index].parent) connection->push_back(nodes[index].pose);
  std::reverse(connection->begin(), connection->end());
  connection->push_back(goal);
  return true;
}
}  // avp 命名空间
