#include "tools/scene_rendering.h"

#include <utility>

namespace avp::tools {
namespace {

QPointF ScenePoint(const Vec2& point) { return {point.x, -point.y}; }

}  // namespace

QPolygonF OrientedBoxPolygon(const Pose2d& pose, double length_m, double width_m) {
  const Vec2 heading = HeadingAxis(pose.yaw);
  const Vec2 lateral = LateralAxis(pose.yaw);
  QPolygonF polygon;
  for (const std::pair<double, double>& corner :
       {std::pair{0.5, 0.5}, {0.5, -0.5}, {-0.5, -0.5}, {-0.5, 0.5}}) {
    polygon << ScenePoint({pose.position.x + corner.first * length_m * heading.x +
                               corner.second * width_m * lateral.x,
                           pose.position.y + corner.first * length_m * heading.y +
                               corner.second * width_m * lateral.y});
  }
  return polygon;
}

QPolygonF VehicleSafetyPolygon(const Pose2d& pose, const VehicleConfig& vehicle) {
  const double margin_diameter = 2.0 * vehicle.safety_margin_m;
  return OrientedBoxPolygon(pose, vehicle.length_m + margin_diameter,
                            vehicle.width_m + margin_diameter);
}

QPen ThinCosmeticPen(const QColor& color, Qt::PenStyle style) {
  QPen pen(color, 1.0, style);
  pen.setCosmetic(true);
  return pen;
}

}  // namespace avp::tools
