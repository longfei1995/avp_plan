#include "speed/speed_planner.h"

#include <array>
#include <cmath>
#include <limits>
#include <map>

namespace avp {
namespace {
// 得到障碍物某时刻的预测位姿
// param: timestamp_ns 绝对时间戳，单位纳秒
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
// 判断s-t图中的点是否与障碍物碰撞
bool IsBlocked(const PlanningFrame& frame, const PathPoint& point, uint64_t timestamp_ns) {
  const Pose2d pose{point.position, point.yaw};
  for (const Obstacle& obstacle : frame.obstacles) {
    if (IsVehicleObstacleCollision(pose, frame.vehicle, PredictAt(obstacle, timestamp_ns).pose,
                                   obstacle.length_m, obstacle.width_m)) {
      return true;
    }
  }
  return false;
}

using StateKey = std::array<int, 3>;
using StateCosts = std::map<StateKey, double>;
using StateParents = std::map<StateKey, int>;

/**
  @brief 状态转移代价，计算从一个 S-T 状态走到下一个 S-T 状态所新增的代价
  @param speed_mps 当前速度，单位米每秒
  @param acceleration_mps2 当前加速度，单位米每平方秒
  @param jerk_mps3 当前 jerk，单位米每立方秒
  @param desired_speed_mps 期望速度，单位米每秒
  @param delta_s 当前路径点与前一状态路径点的距离差，单位米
  @param jerk_weight jerk 平方项的代价权重
  @return 状态转移代价

  @note 代价函数由四部分组成：
  1. 速度跟踪代价：0.2 * (speed_mps - desired_speed_mps)^2
     (这项防止车辆在没有障碍物时无故停着或慢吞吞地走)
  2. 距离代价：0.02 * (delta_s)^2 (会惩罚一个时间步内走得特别远，也就是抑制过高速度)
  3. 加速度代价：0.05 * (acceleration_mps2)^2
     (会惩罚一个时间步内加速度过大，抑制过快的加速或减速)
  4. jerk 代价：jerk_weight * (jerk_mps3)^2 (抑制相邻时间步的加速度突变)
*/
double TransitionCost(double speed_mps, double acceleration_mps2, double jerk_mps3,
                      double desired_speed_mps, double delta_s, double jerk_weight) {
  return 0.2 * std::pow(speed_mps - desired_speed_mps, 2.0) + 0.02 * std::pow(delta_s, 2.0) +
         0.05 * std::pow(acceleration_mps2, 2.0) + jerk_weight * std::pow(jerk_mps3, 2.0);
}

// 纵向运动约束：判断一次状态转移是否满足纵向可行性
bool IsLongitudinalTransitionFeasible(const PlanningFrame& frame, double speed_mps,
                                      double acceleration_mps2, double jerk_mps3) {
  constexpr double kConstraintTolerance = 1e-9;
  return speed_mps <= frame.vehicle.max_speed_mps + kConstraintTolerance &&
         acceleration_mps2 <= frame.vehicle.max_acceleration_mps2 + kConstraintTolerance &&
         acceleration_mps2 >= -frame.vehicle.max_deceleration_mps2 - kConstraintTolerance &&
         jerk_mps3 <= frame.vehicle.max_jerk_mps3 + kConstraintTolerance &&
         jerk_mps3 >= frame.vehicle.min_jerk_mps3 - kConstraintTolerance;
}

}  // namespace

bool SpeedPlanner::Plan(const PlanningFrame& frame, const std::vector<PathPoint>& path,
                        std::vector<SpeedPoint>* profile, std::string* error) const {
  if (profile == nullptr || error == nullptr || path.empty()) {
    return false;
  }
  // 计算时间步数和路径点数
  const int time_count =
      static_cast<int>(std::floor(frame.config.horizon_s / frame.config.time_step_s)) + 1;
  const int path_count = static_cast<int>(path.size());
  if (time_count < 2) {
    *error = "speed horizon must contain at least two time steps";
    return false;
  }
  if (IsBlocked(frame, path.front(), frame.header.timestamp_ns)) {
    *error = "initial S-T lattice state is blocked";
    return false;
  }
  // 1. 初始化常量
  const double infinity = std::numeric_limits<double>::infinity();
  const double time_step = frame.config.time_step_s;
  // todo,保守的速度限制，后续可以根据车道线曲率和障碍物位置进行动态调整
  const double desired_speed = std::min(frame.vehicle.max_speed_mps, 1.5);
  // 2. 从 t0 到 t1 的第一层 DP
  std::vector<double> first_cost(path_count, infinity);
  for (int current = 0; current < path_count; ++current) {
    const double delta_s = path[current].s - path.front().s;
    const double speed = delta_s / time_step;
    const double acceleration = (speed - frame.ego.speed_mps) / time_step;
    const double jerk = (acceleration - frame.ego.acceleration_mps2) / time_step;
    const uint64_t timestamp = frame.header.timestamp_ns + static_cast<uint64_t>(time_step * 1e9);
    if (!IsLongitudinalTransitionFeasible(frame, speed, acceleration, jerk) ||
        IsBlocked(frame, path[current], timestamp)) {
      // 如果候选点超过速度、加速度或 jerk 边界 或者
      // 候选点在 t1 时刻与障碍物碰撞，则该点不可行，跳过
      continue;
    }
    first_cost[current] =
        TransitionCost(speed, acceleration, jerk, desired_speed, delta_s, frame.config.jerk_weight);
  }
  // 3. 初始化到 t=2：构造第一个三元(t0, t1, t2) DP 状态
  StateCosts state_cost;                          // 存储每个三元状态的最小代价
  std::vector<StateParents> parents(time_count);  // 各时间层的回溯记录
  if (time_count >= 3) {
    const uint64_t timestamp =
        frame.header.timestamp_ns + static_cast<uint64_t>(2.0 * time_step * 1e9); // t2 时刻的绝对时间戳
    // 枚举 t1 的位置
    for (int previous = 0; previous < path_count; ++previous) {
      if (!std::isfinite(first_cost[previous])) {
        // 如果起点无法合法到达 previous，这个中间状态不用继续扩展
        continue;
      }
      const double previous_speed = (path[previous].s - path.front().s) / time_step;
      const double previous_acceleration = (previous_speed - frame.ego.speed_mps) / time_step;
      // 枚举 t2 的位置，从previous开始，意味着不允许倒车
      for (int current = previous; current < path_count; ++current) {
        const double delta_s = path[current].s - path[previous].s;
        const double speed = delta_s / time_step;
        if (speed > frame.vehicle.max_speed_mps + 1e-9) {
          // 如果速度超过车辆最大速度，后续的点也会更远，速度只会更大，所以直接跳出循环
          break;
        }
        const double acceleration = (speed - previous_speed) / time_step;
        const double jerk = (acceleration - previous_acceleration) / time_step;
        if (!IsLongitudinalTransitionFeasible(frame, speed, acceleration, jerk) ||
            IsBlocked(frame, path[current], timestamp)) {
          continue;
        }
        // 构建状态{t0 的路径索引, t1 的路径索引, t2 的路径索引}
        const StateKey key{0, previous, current};
        state_cost[key] =
            first_cost[previous] + TransitionCost(speed, acceleration, jerk, desired_speed, delta_s,
                                                  frame.config.jerk_weight);
      }
    }
  }
  // 4. 主循环：从 t3 到 tN 的 DP 扩展
  for (int time_index = 3; time_index < time_count; ++time_index) {
    const uint64_t timestamp =
        frame.header.timestamp_ns + static_cast<uint64_t>(time_index * time_step * 1e9);
    StateCosts next_cost; // 新建下一时间层的 DP 表
    // 遍历上层所有可达状态
    for (const auto& [state, prior_cost] : state_cost) {
      const int before_previous = state[0];
      const int previous = state[1];
      const int current = state[2];
      const double previous_speed = (path[previous].s - path[before_previous].s) / time_step;
      const double current_speed = (path[current].s - path[previous].s) / time_step;
      const double current_acceleration = (current_speed - previous_speed) / time_step;
      // 枚举下一时间节点 s_k+1, 从 current 开始，意味着不允许倒车
      for (int next = current; next < path_count; ++next) {
        const double delta_s = path[next].s - path[current].s;
        const double speed = delta_s / time_step;
        if (speed > frame.vehicle.max_speed_mps + 1e-9) {
          break;
        }
        const double acceleration = (speed - current_speed) / time_step;
        const double jerk = (acceleration - current_acceleration) / time_step;
        if (!IsLongitudinalTransitionFeasible(frame, speed, acceleration, jerk) ||
            IsBlocked(frame, path[next], timestamp)) {
          continue;
        }
        const double candidate =
            prior_cost + TransitionCost(speed, acceleration, jerk, desired_speed, delta_s,
                                        frame.config.jerk_weight);
        // 状态向前“滑动一个时间步”
        // 旧状态：{s(k-2), s(k-1), sk} => 新状态：{s(k-1), sk, s(k+1)}
        const StateKey next_state{previous, current, next};
        const auto next_iter = next_cost.find(next_state);
        if (next_iter == next_cost.end() || candidate < next_iter->second) {
          // 若首次到达，或者当前候选代价更低，就保留当前方案
          next_cost[next_state] = candidate;
          parents[time_index][next_state] = before_previous;
        }
      }
    }
    // 当前时间层扩展完毕后，将新层替换为当前层
    state_cost = std::move(next_cost);
  }
  // 5. 终点选择：平顺性与前进距离的权衡
  double final_cost = infinity;
  int final_first_index = -1;
  StateKey final_state{-1, -1, -1};
  if (time_count == 2) {
    // 特殊处理只有 t0,t1 两层的情况
    for (int current = 0; current < path_count; ++current) {
      const double cost = first_cost[current];
      const double progress_reward = 0.1 * path[current].s;
      if (cost - progress_reward < final_cost) {
        // 更远的可行终点会获得额外奖励
        final_cost = cost - progress_reward;
        final_first_index = current;
      }
    }
  } else {  // 一般情况中，遍历最后时间层的所有三元状态
    for (const auto& [state, cost] : state_cost) {
      const double progress_reward = 0.1 * path[state[2]].s;
      if (cost - progress_reward < final_cost) {
        final_cost = cost - progress_reward;
        final_state = state;
      }
    }
  }
  if (!std::isfinite(final_cost)) {
    // 所有同时满足速度、加速度、jerk 约束的 S-T 轨迹，都被障碍物或约束淘汰了
    *error = "all acceleration- and jerk-feasible S-T lattice states are blocked";
    return false;
  }
  // 6. 回溯：从终点状态回溯到起点，得到最优路径索引序列
  std::vector<int> indices(time_count); // 用于保存每个时间层最终选到的路径索引
  if (time_count == 2) {
    indices[0] = 0;
    indices[1] = final_first_index;
  } else {
    indices[time_count - 3] = final_state[0];
    indices[time_count - 2] = final_state[1];
    indices[time_count - 1] = final_state[2];
    StateKey state = final_state;
    // 从最后时刻逐层回溯，直到第 3 层
    for (int time_index = time_count - 1; time_index >= 3; --time_index) {
      const auto parent_iter = parents[time_index].find(state);
      if (parent_iter == parents[time_index].end()) {
        // 理论上不应发生。若发生，说明 DP 代价表与父节点表不一致。
        *error = "S-T lattice backtracking failed";
        return false;
      }
      const int before = parent_iter->second;
      indices[time_index - 3] = before;
      state = {before, state[0], state[1]};
    }
  }
  if (indices.front() != 0) {
    *error = "S-T lattice does not start at the path origin";
    return false;
  }
  // 7. 生成最终速度曲线
  profile->clear();
  profile->reserve(time_count);
  double previous_speed = frame.ego.speed_mps;
  for (int time_index = 0; time_index < time_count; ++time_index) {
    const double speed =
        time_index == 0
            ? frame.ego.speed_mps
            : (path[indices[time_index]].s - path[indices[time_index - 1]].s) / time_step;
    const double acceleration =
        time_index == 0 ? frame.ego.acceleration_mps2 : (speed - previous_speed) / time_step;
    profile->push_back({time_index * time_step, path[indices[time_index]].s, speed, acceleration});
    previous_speed = speed;
  }
  return true;
}
}  // namespace avp
