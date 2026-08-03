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
  Mode mode_ = Mode::kLaneApproach;
  std::string active_target_parking_spot_id_;
  ParkingManeuver parking_maneuver_;
  size_t parking_segment_index_ = 0;
  uint64_t gear_shift_start_timestamp_ns_ = 0;
  uint64_t last_parking_replan_timestamp_ns_ = 0;
  DrivingDirection pending_direction_ = DrivingDirection::kUnknown;
};
}  // avp 命名空间
#endif  // 头文件保护
