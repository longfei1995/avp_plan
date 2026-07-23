#ifndef AVP_SPEED_SPEED_PLANNER_H_
#define AVP_SPEED_SPEED_PLANNER_H_

#include <string>
#include <vector>
#include "common/types.h"
namespace avp {
struct SpeedPoint { double time_s = 0.0; double s = 0.0; double speed_mps = 0.0; double acceleration_mps2 = 0.0; };
class SpeedPlanner {
 public:
  bool Plan(const PlanningFrame& frame, const std::vector<PathPoint>& path,
            std::vector<SpeedPoint>* profile, std::string* error) const;
};
}  // avp 命名空间
#endif  // 头文件保护
