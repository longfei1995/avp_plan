#include "local/local_planner.h"

#include <cmath>
#include <limits>

namespace avp {
namespace {
// 局部规划器内部使用的参考线点
struct ReferencePoint {
  Vec2 position;        // 二维坐标
  double yaw = 0.0;     // 该点参考线的切线方向
  double s = 0.0;       // 从参考线起点累计的长度
};
// 根据连续三个点估计中间点 b 的曲率。
double Curvature(const Vec2& a, const Vec2& b, const Vec2& c) {
  const double ab = Distance(a, b);
  const double bc = Distance(b, c);
  const double ac = Distance(a, c);
  if (ab < 1e-6 || bc < 1e-6 || ac < 1e-6) {
    return 0.0;
  }
  const double area2 = std::abs(((b.x - a.x) * (c.y - a.y)) -
                                ((b.y - a.y) * (c.x - a.x)));
  return 2.0 * area2 / (ab * bc * ac);
}
// 按到达时刻取障碍物预测位置
PredictionPoint PredictAt(const Obstacle& obstacle, uint64_t timestamp_ns) {
  PredictionPoint result = obstacle.prediction.front();
  for (const PredictionPoint& point : obstacle.prediction) {
    if (point.timestamp_ns > timestamp_ns) {
      break;
    }
    result = point;
  }
  return result;
}
// 重采样参考线，保证相邻点间距不大于 step_m。
std::vector<ReferencePoint> ResampleReferenceLine(const std::vector<Vec2>& reference_line,
                                                  double step_m) {
  std::vector<Vec2> positions{reference_line.front()};
  for (size_t i = 1; i < reference_line.size(); ++i) {
    const Vec2& start = reference_line[i - 1];
    const Vec2& end = reference_line[i];
    const double length = Distance(start, end);
    if (length < 1e-6) {
      continue;
    }
    const int count = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    for (int sample = 1; sample <= count; ++sample) {
      positions.push_back(Interpolate(start, end, static_cast<double>(sample) / count));
    }
  }

  std::vector<ReferencePoint> result;
  result.reserve(positions.size());
  double s = 0.0;
  for (size_t i = 0; i < positions.size(); ++i) {
    if (i > 0) {
      s += Distance(positions[i - 1], positions[i]);
    }
    const Vec2& before = positions[i == 0 ? 0 : i - 1];
    const Vec2& after = positions[i + 1 < positions.size() ? i + 1 : i];
    result.push_back({positions[i], std::atan2(after.y - before.y, after.x - before.x), s});
  }
  return result;
}
// 从参考线向左/右偏移
Vec2 OffsetPosition(const ReferencePoint& reference, double lateral_offset_m) {
  return {reference.position.x - std::sin(reference.yaw) * lateral_offset_m,
          reference.position.y + std::cos(reference.yaw) * lateral_offset_m};
}
// 时空碰撞检查
bool IsCollisionFree(const PlanningFrame& frame, const Pose2d& pose, double arrival_time_s) {
  // 把相对的到达时间戳 => 绝对时间戳(ns)
  const uint64_t timestamp = frame.header.timestamp_ns +
                             static_cast<uint64_t>(arrival_time_s * 1e9);
  for (const Obstacle& obstacle : frame.obstacles) {
    if (IsVehicleObstacleCollision(pose, frame.vehicle, PredictAt(obstacle, timestamp).pose,
                                   obstacle.length_m, obstacle.width_m)) {
      return false;
    }
  }
  return true;
}

// 首轮路径规划没有速度剖面，所有路径层均使用当前时刻；后续轮次使用上一轮速度规划
// 回投的到达时刻。索引超出范围时沿用最后一个有效时刻，防止数组越界。
double ArrivalTimeAt(const std::vector<double>& arrival_times, size_t index) {
  if (arrival_times.empty()) {
    return 0.0;
  }
  return arrival_times[std::min(index, arrival_times.size() - 1)];
}

// 将一对横向索引映射到格点状态数组下标。
size_t StateIndex(int previous_lateral_index, int current_lateral_index, int lateral_count) {
  return static_cast<size_t>(previous_lateral_index) * static_cast<size_t>(lateral_count) +
         static_cast<size_t>(current_lateral_index);
}
}; // namespace

bool LocalPlanner::Plan(const PlanningFrame& frame, const GlobalRoute& route,
                        const std::vector<double>& arrival_times, std::vector<PathPoint>* path,
                        std::string* error) const {
  if (path == nullptr || error == nullptr || route.reference_line.size() < 2) {
    return false;
  }
  // 1. 重采样参考线，并把自车作为路径起点
  // todo 如果全局参考线首点离自车很远，这会产生一条很长的第一段连接线。
  // 真实工程中通常会先把自车投影到全局参考线上，再截取附近一段参考线，而不是简单插入一个点。
  std::vector<ReferencePoint> reference =
      ResampleReferenceLine(route.reference_line, frame.config.path_step_m);
  if (reference.size() < 2) {
    *error = "reference line has no usable length";
    return false;
  }
  if (Distance(reference.front().position, frame.ego.pose.position) < 1e-6) {
    // 严格替换为自车状态
    reference.front().position = frame.ego.pose.position;
    reference.front().yaw = frame.ego.pose.yaw;
    reference.front().s = 0.0;
  } else {
    reference.insert(reference.begin(), {frame.ego.pose.position, frame.ego.pose.yaw, 0.0});
  }
  for (size_t index = 1; index < reference.size(); ++index) {
    reference[index].s =
        reference[index - 1].s + Distance(reference[index - 1].position, reference[index].position);
  }
  // 2. 定义横向采样范围
  constexpr int kLateralCount = 13;           // 每层横向采样 13 个候选点
  constexpr double kLateralStepM = 0.3;       // 横向采样间距 0.3 m
  // DP 的三个代价权重
  constexpr double kLateralWeight = 1.0;      // 偏离参考线惩罚
  constexpr double kSlopeWeight = 4.0;        // 横向变化过快的惩罚
  constexpr double kCurvatureWeight = 12.0;   // 曲率过大惩罚
  // lateral_offsets 保存 13 个横向偏移值，单位 m
  std::vector<double> lateral_offsets;
  lateral_offsets.reserve(kLateralCount);
  for (int index = 0; index < kLateralCount; ++index) {
    const double centered_index = static_cast<double>(index) -
                                  static_cast<double>(kLateralCount - 1) / 2.0;
    lateral_offsets.push_back(centered_index * kLateralStepM);
  }
  // 3. 构建 S-L 位姿候选表格 positions
  std::vector<std::vector<Vec2>> positions(reference.size(),
                                           std::vector<Vec2>(kLateralCount));
  for (size_t layer = 0; layer < reference.size(); ++layer) {
    for (int lateral = 0; lateral < kLateralCount; ++lateral) {
      positions[layer][lateral] = OffsetPosition(reference[layer], lateral_offsets[lateral]);
    }
  }
  for (int lateral = 0; lateral < kLateralCount; ++lateral) {
    positions[0][lateral] = frame.ego.pose.position;
  }
  // 4. 初始化 DP 
  // 4.1 初始化第 0 层
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> initial_cost(kLateralCount, infinity);
  const int anchor_lateral = kLateralCount / 2;   // 横向偏移为 0 的中心索引
  if (IsCollisionFree(frame, frame.ego.pose, ArrivalTimeAt(arrival_times, 0))) {
    // 如果自车在当前时刻没有和障碍物碰撞，则第 0 层中心点可达，总代价为 0。
    initial_cost[anchor_lateral] = 0.0;
  }
  // 4.2 初始化第 1 层
  const double first_distance = Distance(reference[0].position, reference[1].position);
  if (first_distance < 1e-6) {
    // 如果参考线前两点重合，朝向可能无意义，曲率可能无意义，无法计算 DP 代价。
    *error = "reference line contains duplicate samples";
    return false;
  }
  // 4.3 把0-1层的状态写入DP状态表state_cost
  const size_t lattice_state_count = static_cast<size_t>(kLateralCount) * kLateralCount;
  std::vector<double> state_cost(lattice_state_count, infinity);
  for (int first = 0; first < kLateralCount; ++first) {
    if (!std::isfinite(initial_cost[first])) {
      // 实际上只有 first == 6 能进入下一步
      continue;
    }
    for (int second = 0; second < kLateralCount; ++second) {
      // delta 是从自车点到第 1 层候选点的位移向量
      const Vec2 delta{positions[1][second].x - positions[0][first].x,
                       positions[1][second].y - positions[0][first].y};
      const Pose2d pose{positions[1][second], std::atan2(delta.y, delta.x)};
      const double initial_connection_length = Distance(positions[0][first], positions[1][second]);
      // 首段的曲率可行性检查 ∣Δθ∣≤κmax​Δs
      if (std::abs(NormalizeAngle(pose.yaw - frame.ego.pose.yaw)) >
          frame.vehicle.max_curvature_1pm * initial_connection_length + 1e-9) {
        continue;
      }
      // 检查自车在预计到达第 1 点的时刻，是否会与障碍物碰撞。
      if (!IsCollisionFree(frame, pose, ArrivalTimeAt(arrival_times, 1))) {
        continue;
      }
      // 横向变化率 dl / ds，表征 “沿参考线前进时，横向偏移改变得有多快”
      const double lateral_rate = (lateral_offsets[second] - lateral_offsets[first]) / first_distance;
      state_cost[StateIndex(first, second, kLateralCount)] =
          initial_cost[first] + kLateralWeight * lateral_offsets[second] * lateral_offsets[second] +
          kSlopeWeight * lateral_rate * lateral_rate;
    }
  }
  // 父状态数组 parents 用于回溯最优路径
  std::vector<std::vector<int>> parents(reference.size(),
                                         std::vector<int>(lattice_state_count, -1));
  // 5. DP 主循环：从第 2 层逐层向前扩展
  for (size_t layer = 2; layer < reference.size(); ++layer) {
    // 一共有三个状态：前一层 previous，当前层 current，下一层 next
    // current 是 layer-1，next 是 layer，previous 是 layer-2
    std::vector<double> next_cost(lattice_state_count, infinity);  // 保存(layer-1, layer)的状态代价
    const double distance = Distance(positions[layer - 1][0], positions[layer][0]);
    if (distance < 1e-6) {
      *error = "reference line contains duplicate samples";
      return false;
    }
    for (int previous = 0; previous < kLateralCount; ++previous) {
      for (int current = 0; current < kLateralCount; ++current) {
        const double prior_cost = state_cost[StateIndex(previous, current, kLateralCount)];
        if (!std::isfinite(prior_cost)) {
          continue;
        }
        for (int next = 0; next < kLateralCount; ++next) {
          const Vec2 delta{positions[layer][next].x - positions[layer - 1][current].x,
                           positions[layer][next].y - positions[layer - 1][current].y};
          const Pose2d pose{positions[layer][next], std::atan2(delta.y, delta.x)};
          if (!IsCollisionFree(frame, pose, ArrivalTimeAt(arrival_times, layer))) {
            // 硬约束：禁止碰撞
            continue;
          }
          // 计算current的曲率，也就是 layer-1 层的曲率
          const double curvature = Curvature(positions[layer - 2][previous],
                                             positions[layer - 1][current], positions[layer][next]);
          if (curvature > frame.vehicle.max_curvature_1pm) {
            // 硬约束：禁止曲率超限
            continue;
          }
          const double lateral_rate = (lateral_offsets[next] - lateral_offsets[current]) / distance;  // dl/ds
          const double candidate = prior_cost +
                                   kLateralWeight * lateral_offsets[next] * lateral_offsets[next] +
                                   kSlopeWeight * lateral_rate * lateral_rate +
                                   kCurvatureWeight * curvature * curvature;
          const size_t next_state = StateIndex(current, next, kLateralCount);
          if (candidate < next_cost[next_state]) {
            // DP 最优子结构，记录最优代价和父状态
            next_cost[next_state] = candidate;
            parents[layer][next_state] = previous;
          }
        }
      }
    }
    state_cost = std::move(next_cost);  // 内部资源转移给 state_cost，避免不必要的大数组拷贝
  }
  // 6. 回溯最优路径
  // 6.1 在最后一层寻找总代价最小的终点状态
  int previous = -1;      // 倒数第二层的横向索引
  int current = -1;       // 最后一层的横向索引
  double best_cost = infinity;
  for (int first = 0; first < kLateralCount; ++first) {
    for (int second = 0; second < kLateralCount; ++second) {
      // 最后一层的所有状态(first, second)
      // 没有强制最后一个点回到中心线，只是代价函数会自然偏好较小横向偏移。
      // 因此如果障碍物要求绕行，最终点可能仍然停在左侧或右侧。
      const double cost = state_cost[StateIndex(first, second, kLateralCount)];
      if (cost < best_cost) {
        best_cost = cost;
        previous = first;
        current = second;
      }
    }
  }
  if (!std::isfinite(best_cost)) {
     // 终所有状态仍然是无穷大，说明没有一条完整可行路径。
    *error = "no feasible S-L lattice path";
    return false;
  }
  // 6.2 根据 parents 回溯每层横向索引
  std::vector<int> lateral_indices(reference.size()); // 保存每一层到底选择横向 0 到 12 中的哪一个。
  lateral_indices.back() = current;
  lateral_indices[reference.size() - 2] = previous;
  for (size_t layer = reference.size() - 1; layer >= 2; --layer) {
    const int before = parents[layer][StateIndex(previous, current, kLateralCount)];
    if (before < 0) {
      *error = "S-L lattice backtracking failed";
      return false;
    }
    lateral_indices[layer - 2] = before;
    current = previous;
    previous = before;
  }
  // 6.3 将横向索引转换为最终 PathPoint
  path->clear();
  path->reserve(reference.size());
  double s = 0.0;
  for (size_t index = 0; index < reference.size(); ++index) {
    const Vec2 position = positions[index][lateral_indices[index]];
    if (index > 0) {
      // 从第二个点开始，累加相邻路径点的距离
      s += Distance(path->back().position, position);
    }
    // 取路径上的前后点计算切线方向，保证路径平滑
    const Vec2& before = index == 0 ? position : path->back().position;
    const Vec2& after = index + 1 < reference.size()
                            ? positions[index + 1][lateral_indices[index + 1]]
                            : position;
    // 中间点，使用前后点的中心差分方向， 比如第 1 个点，使用第 0 个点和第 2 个点的方向。
    const double yaw = index == 0
                           ? frame.ego.pose.yaw
                           : (index + 1 < reference.size()
                                  ? std::atan2(after.y - before.y, after.x - before.x)
                                  : path->back().yaw);
    path->push_back({position, yaw, 0.0, s});
  }
  // 重新计算输出路径曲率
  if (path->size() > 1) {
    // 首点曲率计算
    const double first_segment = Distance(path->at(0).position, path->at(1).position);
    const double first_segment_yaw =
        std::atan2(path->at(1).position.y - path->at(0).position.y,
                   path->at(1).position.x - path->at(0).position.x);
    path->at(0).curvature =
        std::abs(NormalizeAngle(first_segment_yaw - path->at(0).yaw)) / first_segment;
  }
  // 所有中间点曲率计算，不包括首尾点
  for (size_t index = 1; index + 1 < path->size(); ++index) {
    path->at(index).curvature = Curvature(path->at(index - 1).position, path->at(index).position,
                                          path->at(index + 1).position);
  }
  // todo, 缺少对终点曲率的处理
  // 最终输出路径逐点复检
  for (size_t index = 0; index < path->size(); ++index) {
    if (path->at(index).curvature > frame.vehicle.max_curvature_1pm ||
        !IsCollisionFree(frame, {path->at(index).position, path->at(index).yaw},
                         ArrivalTimeAt(arrival_times, index))) {
      *error = "selected S-L lattice path is invalid";
      path->clear();
      return false;
    }
  }
  return true;
}
}  // avp 命名空间
