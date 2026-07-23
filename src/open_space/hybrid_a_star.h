#ifndef AVP_OPEN_SPACE_HYBRID_A_STAR_H_
#define AVP_OPEN_SPACE_HYBRID_A_STAR_H_

#include <string>
#include <vector>

#include "common/types.h"

namespace avp {
class HybridAStar {
 public:
  bool Plan(const PlanningFrame& frame, const Pose2d& start, const Pose2d& goal,
            std::vector<Pose2d>* connection, std::string* error) const;
};
}  // avp 命名空间

#endif  // 头文件保护
