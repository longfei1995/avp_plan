#ifndef AVP_OPEN_SPACE_HYBRID_A_STAR_H_
#define AVP_OPEN_SPACE_HYBRID_A_STAR_H_

#include <string>
#include <vector>

#include "common/types.h"

namespace avp {
struct HybridPathPoint {
  Pose2d pose;
  double signed_curvature_1pm = 0.0;
  DrivingDirection direction = DrivingDirection::kUnknown;
};

struct ParkingSegment {
  DrivingDirection direction = DrivingDirection::kUnknown;
  std::vector<HybridPathPoint> points;
};

struct ParkingManeuver {
  std::vector<ParkingSegment> segments;
};

class HybridAStar {
 public:
  bool Plan(const PlanningFrame& frame, const Pose2d& start, const Pose2d& goal,
            ParkingManeuver* maneuver, std::string* error) const;
};
}  // avp 命名空间

#endif  // 头文件保护
