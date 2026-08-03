#ifndef AVP_PLANNING_PLANNER_H_
#define AVP_PLANNING_PLANNER_H_

#include "common/types.h"
#include "interfaces/adapters.h"
#include "local/local_planner.h"
#include "map/global_planner.h"
#include "open_space/hybrid_a_star.h"
#include "speed/speed_planner.h"
namespace avp {
class Planner {
 public:
  explicit Planner(VehicleConfig vehicle = {}, PlannerConfig config = {});
  PlanningResponse Plan(const PlanningRequest& request);

 private:
  enum class Mode { kLaneApproach, kOpenSpaceParking, kGearShift };

  PlanningResponse PlanParking(const PlanningFrame& frame);
  void ResetTask(const std::string& target_parking_spot_id);

  VehicleConfig vehicle_;
  PlannerConfig config_;
  PlanningFrameAdapter adapter_;
  GlobalPlanner global_planner_;
  LocalPlanner local_planner_;
  SpeedPlanner speed_planner_;
  HybridAStar hybrid_a_star_;
  Mode mode_ = Mode::kLaneApproach;              // 内部状态机
  std::string active_target_parking_spot_id_;    // 当前激活的车位id，用于检测任务是否切换
  ParkingManeuver parking_maneuver_;             // Hybrid A* 生成的完整泊车动作，包含一个或多个同向行驶段。
  size_t parking_segment_index_ = 0;             // 当前正在执行的泊车段的下标
  uint64_t gear_shift_start_timestamp_ns_ = 0;   // 换挡开始时刻时间戳
  uint64_t last_parking_replan_timestamp_ns_ = 0;// 最近一次泊车重规划时间戳
  DrivingDirection pending_direction_ = DrivingDirection::kUnknown;  // 行驶方向
};
}  // avp 命名空间
#endif  // 头文件保护
