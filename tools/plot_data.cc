#include "tools/plot_data.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace avp::tools {
namespace {

std::vector<double> AccumulatedTrajectoryS(const PlanningResponse& response) {
  std::vector<double> result(response.trajectory.size(), 0.0);
  for (size_t index = 1; index < response.trajectory.size(); ++index) {
    result[index] = result[index - 1] +
                    Distance(response.trajectory[index - 1].pose.position,
                             response.trajectory[index].pose.position);
  }
  return result;
}

PlotPoint ProjectToReference(const Vec2& position, const std::vector<Vec2>& reference_line,
                             const std::vector<double>& reference_s) {
  double best_distance_squared = std::numeric_limits<double>::infinity();
  PlotPoint best;
  for (size_t index = 1; index < reference_line.size(); ++index) {
    const Vec2& start = reference_line[index - 1];
    const Vec2& end = reference_line[index];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared <= 1e-12) continue;
    const double ratio = std::clamp(
        ((position.x - start.x) * dx + (position.y - start.y) * dy) / length_squared,
        0.0, 1.0);
    const Vec2 projection{start.x + ratio * dx, start.y + ratio * dy};
    const double offset_x = position.x - projection.x;
    const double offset_y = position.y - projection.y;
    const double distance_squared = offset_x * offset_x + offset_y * offset_y;
    if (distance_squared >= best_distance_squared) continue;
    const double length = std::sqrt(length_squared);
    best_distance_squared = distance_squared;
    best.x = reference_s[index - 1] + ratio * length;
    best.y = (dx * offset_y - dy * offset_x) / length;
  }
  return best;
}

std::vector<PlotPoint> BuildSlPath(const PlanningDebugData& debug) {
  if (debug.planning_mode != "LANE_APPROACH" || debug.coupling_iterations.empty() ||
      debug.cropped_reference_line.size() < 2) {
    return {};
  }
  std::vector<double> reference_s(debug.cropped_reference_line.size(), 0.0);
  for (size_t index = 1; index < debug.cropped_reference_line.size(); ++index) {
    reference_s[index] = reference_s[index - 1] +
                         Distance(debug.cropped_reference_line[index - 1],
                                  debug.cropped_reference_line[index]);
  }
  std::vector<PlotPoint> result;
  const auto& path = debug.coupling_iterations.back().local_path;
  result.reserve(path.size());
  for (const PathPoint& point : path) {
    result.push_back(ProjectToReference(point.position, debug.cropped_reference_line, reference_s));
  }
  return result;
}

}  // namespace

PlanningPlotData BuildPlanningPlotData(const PlanningDebugData& debug,
                                       const PlanningResponse& response) {
  PlanningPlotData result;
  result.sl_path = BuildSlPath(debug);
  if (debug.planning_mode == "OPEN_SPACE_PARKING" || debug.planning_mode == "GEAR_SHIFT") {
    result.sl_empty_message = "S-L is only available during lane approach";
  } else if (result.sl_path.empty()) {
    result.sl_empty_message = "No S-L path";
  }

  const std::vector<double> accumulated_s = AccumulatedTrajectoryS(response);
  result.st_path.reserve(response.trajectory.size());
  result.curvature_by_s.reserve(response.trajectory.size());
  result.yaw_by_s.reserve(response.trajectory.size());
  double unwrapped_yaw = 0.0;
  double previous_yaw = 0.0;
  for (size_t index = 0; index < response.trajectory.size(); ++index) {
    const TimedTrajectoryPoint& point = response.trajectory[index];
    result.st_path.push_back({point.relative_time_s, accumulated_s[index]});
    result.curvature_by_s.push_back({accumulated_s[index], point.curvature_1pm});
    if (index == 0) {
      unwrapped_yaw = point.pose.yaw;
    } else {
      unwrapped_yaw += NormalizeAngle(point.pose.yaw - previous_yaw);
    }
    previous_yaw = point.pose.yaw;
    result.yaw_by_s.push_back({accumulated_s[index], unwrapped_yaw * 180.0 / kPi});
  }
  return result;
}

}  // namespace avp::tools
