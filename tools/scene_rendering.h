#ifndef AVP_TOOLS_SCENE_RENDERING_H_
#define AVP_TOOLS_SCENE_RENDERING_H_

#include <QColor>
#include <QPen>
#include <QPolygonF>

#include "common/types.h"

class QGraphicsEllipseItem;
class QGraphicsScene;

namespace avp::tools {

inline constexpr double kParkingEntryMarkerDiameterPx = 8.0;

QPolygonF OrientedBoxPolygon(const Pose2d& pose, double length_m, double width_m);
QPolygonF VehicleSafetyPolygon(const Pose2d& pose, const VehicleConfig& vehicle);
QPen ThinCosmeticPen(const QColor& color, Qt::PenStyle style = Qt::SolidLine);
QGraphicsEllipseItem* AddParkingEntryMarker(QGraphicsScene* scene, const Vec2& position);

}  // namespace avp::tools

#endif  // AVP_TOOLS_SCENE_RENDERING_H_
