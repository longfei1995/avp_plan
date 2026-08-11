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
  enum class Mode {
    kLaneApproach,      // 车道接近泊车入口阶段
    kOpenSpaceParking,  // 泊车阶段
    kGearShift          // 等待换挡阶段
  };
  /**
    @brief 规划泊车动作
    @param frame 当前规划帧
    @return 规划结果
    @note: 
    开放空间泊车状态下的主逻辑。它负责：
      1. 执行 Hybrid A* 生成的多个泊车段；
      2. 判断是否需要换挡；
      3. 生成当前泊车段的局部路径和速度；
      4. 检测偏离与碰撞；
      5. 失败时重规划或紧急停车。
  */
  PlanningResponse PlanParking(const PlanningFrame& frame);
  // 从当前车辆位姿重新生成泊车动作；同一规划帧最多尝试一次
  bool ReplanParking(const PlanningFrame& frame, std::string* error);
  // 当目标车位改变时，清除上一任务遗留的状态，重新开始
  void ResetTask(const std::string& target_parking_spot_id);

  // 构造时保存的车辆参数和规划配置。每次 Plan() 会把它们传给适配器，形成当前周期的 PlanningFrame
  VehicleConfig vehicle_;
  PlannerConfig config_;
  PlanningFrameAdapter adapter_;
  GlobalPlanner global_planner_;
  LocalPlanner local_planner_;
  SpeedPlanner speed_planner_;
  HybridAStar hybrid_a_star_;
  Mode mode_ = Mode::kLaneApproach;            // 内部状态机
  std::string active_target_parking_spot_id_;  // 当前激活的车位id
  ParkingManeuver parking_maneuver_;  // 完整的泊车动作，通常包含多个 ParkingSegment
  size_t parking_segment_index_ = 0;  // 当前泊车段的下标
  uint64_t gear_shift_start_timestamp_ns_ = 0;  // 开始等待换挡的绝对时间戳
  uint64_t last_parking_replan_timestamp_ns_ = 0; // 上一次重规划的时间戳
  DrivingDirection pending_direction_ = DrivingDirection::kUnknown;  // 即将切换的档位
};
}  // namespace avp
#endif  // 头文件保护
