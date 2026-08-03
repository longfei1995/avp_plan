#pragma once

#include <string>
#include <vector>

#include "common/types.h"
namespace avp {
struct GlobalRoute {
  std::vector<std::string> lane_ids;
  std::vector<Vec2> reference_line;
  Pose2d parking_entry;
  Pose2d parking_target;
};
class GlobalPlanner {
 public:
  bool Plan(const PlanningFrame& frame, GlobalRoute* route, std::string* error) const;

};
}  // namespace avp
