#ifndef AVP_TOOLS_PLOT_DATA_H_
#define AVP_TOOLS_PLOT_DATA_H_

#include <string>
#include <vector>

#include "planning/planner.h"

namespace avp::tools {

struct PlotPoint {
  double x = 0.0;
  double y = 0.0;
};

struct PlanningPlotData {
  std::vector<PlotPoint> sl_path;
  std::vector<PlotPoint> st_path;
  std::vector<PlotPoint> curvature_by_s;
  std::vector<PlotPoint> yaw_by_s;
  std::string sl_empty_message;
};

PlanningPlotData BuildPlanningPlotData(const PlanningDebugData& debug,
                                       const PlanningResponse& response);

}  // namespace avp::tools

#endif  // AVP_TOOLS_PLOT_DATA_H_
