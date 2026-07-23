#ifndef AVP_LOCAL_LOCAL_PLANNER_H_
#define AVP_LOCAL_LOCAL_PLANNER_H_

#include <string>
#include <vector>
#include "common/types.h"
#include "map/global_planner.h"
namespace avp {
class LocalPlanner {
 public:
  bool Plan(const PlanningFrame& frame, const GlobalRoute& route,
            const std::vector<double>& arrival_times, std::vector<PathPoint>* path,
            std::string* error) const;
};
}  // avp 命名空间
#endif  // 头文件保护
