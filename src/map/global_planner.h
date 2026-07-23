#ifndef AVP_MAP_GLOBAL_PLANNER_H_
#define AVP_MAP_GLOBAL_PLANNER_H_

#include <string>
#include <vector>
#include "common/types.h"
#include "open_space/hybrid_a_star.h"
namespace avp {
struct GlobalRoute { std::vector<std::string> lane_ids; std::vector<Vec2> reference_line; Pose2d parking_target; };
class GlobalPlanner {
 public:
  bool Plan(const PlanningFrame& frame, GlobalRoute* route, std::string* error) const;
 private:
  HybridAStar hybrid_a_star_;
};
}  // avp 命名空间
#endif  // 头文件保护
