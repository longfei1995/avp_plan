#ifndef AVP_TOOLS_PLOT_DATA_H_
#define AVP_TOOLS_PLOT_DATA_H_

#include <cstddef>
#include <string>
#include <vector>

#include "common/types.h"

namespace avp::tools {

struct PlotPoint {
  double x = 0.0;
  double y = 0.0;
};

struct ResponseTrajectoryRow {
  size_t index = 0;
  double relative_time_s = 0.0;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double curvature_1pm = 0.0;
  double speed_mps = 0.0;
  double acceleration_mps2 = 0.0;
  std::string gear;
};

std::vector<ResponseTrajectoryRow> BuildResponseTrajectoryRows(
    const PlanningResponse& response);

}  // namespace avp::tools

#endif  // AVP_TOOLS_PLOT_DATA_H_
