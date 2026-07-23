#ifndef AVP_PLANNING_PLANNER_H_
#define AVP_PLANNING_PLANNER_H_

#include "common/types.h"
#include "interfaces/adapters.h"
#include "local/local_planner.h"
#include "map/global_planner.h"
#include "speed/speed_planner.h"
namespace avp {
class Planner {
 public:
  explicit Planner(VehicleConfig vehicle = {}, PlannerConfig config = {});
  PlanningResponse Plan(const PlanningRequest& request) const;
 private:
  VehicleConfig vehicle_;
  PlannerConfig config_;
  PlanningFrameAdapter adapter_;
  GlobalPlanner global_planner_;
  LocalPlanner local_planner_;
  SpeedPlanner speed_planner_;
};
}  // avp 命名空间
#endif  // 头文件保护
