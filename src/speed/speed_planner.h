#pragma once

#include <limits>
#include <string>
#include <vector>

#include "common/types.h"
namespace avp {
struct SpeedPoint {
  double time_s = 0.0;                  // 相对当前规划时刻的时间
  double s = 0.0;                       // 沿局部路径前进的累计距离
  double speed_mps = 0.0;               // 当前时刻的速度
  double acceleration_mps2 = 0.0;       // 当前时刻的加速度
};
struct SpeedPlanOptions {
  double max_speed_mps = std::numeric_limits<double>::infinity();
  bool require_stop_at_end = false;
};
class SpeedPlanner {
 public:
  bool Plan(const PlanningFrame& frame, const std::vector<PathPoint>& path,
            std::vector<SpeedPoint>* profile, std::string* error,
            const SpeedPlanOptions& options = {}) const;
};
}  // namespace avp
