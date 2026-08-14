#include "tools/plot_data.h"

namespace avp::tools {
namespace {

const char* GearLabel(DrivingDirection direction) {
  switch (direction) {
    case DrivingDirection::kDrive:
      return "D";
    case DrivingDirection::kReverse:
      return "R";
    case DrivingDirection::kUnknown:
      return "N";
  }
  return "N";
}

}  // namespace

std::vector<ResponseTrajectoryRow> BuildResponseTrajectoryRows(
    const PlanningResponse& response) {
  std::vector<ResponseTrajectoryRow> result;
  result.reserve(response.trajectory.size());
  for (size_t index = 0; index < response.trajectory.size(); ++index) {
    const TimedTrajectoryPoint& point = response.trajectory[index];
    result.push_back({index,
                      point.relative_time_s,
                      point.pose.position.x,
                      point.pose.position.y,
                      point.pose.yaw,
                      point.curvature_1pm,
                      point.speed_mps,
                      point.acceleration_mps2,
                      GearLabel(point.direction)});
  }
  return result;
}

}  // namespace avp::tools
