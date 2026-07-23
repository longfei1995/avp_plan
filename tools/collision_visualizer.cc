#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include <matplot/matplot.h>

#include "collision_test_cases.h"

namespace {

std::array<avp::Vec2, 5> RectangleCorners(const avp::Pose2d& pose, double length_m,
                                          double width_m) {
  const avp::Vec2 heading = avp::HeadingAxis(pose.yaw);
  const avp::Vec2 lateral = avp::LateralAxis(pose.yaw);
  const double half_length = length_m * 0.5;
  const double half_width = width_m * 0.5;
  const std::array<avp::Vec2, 4> local_corners = {
      avp::Vec2{half_length, half_width}, avp::Vec2{half_length, -half_width},
      avp::Vec2{-half_length, -half_width}, avp::Vec2{-half_length, half_width}};

  std::array<avp::Vec2, 5> corners;
  for (size_t index = 0; index < local_corners.size(); ++index) {
    const avp::Vec2& local_corner = local_corners[index];
    corners[index] = {pose.position.x + heading.x * local_corner.x + lateral.x * local_corner.y,
                      pose.position.y + heading.y * local_corner.x + lateral.y * local_corner.y};
  }
  corners.back() = corners.front();
  return corners;
}

void DrawRectangle(const matplot::axes_handle& axis, const avp::Pose2d& pose, double length_m,
                   double width_m, std::string_view line_spec, float line_width) {
  const std::array<avp::Vec2, 5> corners = RectangleCorners(pose, length_m, width_m);
  std::vector<double> x_coordinates;
  std::vector<double> y_coordinates;
  x_coordinates.reserve(corners.size());
  y_coordinates.reserve(corners.size());
  for (const avp::Vec2& corner : corners) {
    x_coordinates.push_back(corner.x);
    y_coordinates.push_back(corner.y);
  }
  axis->plot(x_coordinates, y_coordinates, line_spec)->line_width(line_width);
}

void DrawHeading(const matplot::axes_handle& axis, const avp::Pose2d& pose, double length_m,
                 std::string_view color) {
  const double arrow_length = std::min(1.0, length_m * 0.35);
  const auto arrow = axis->arrow(pose.position.x, pose.position.y,
                                 arrow_length * std::cos(pose.yaw),
                                 arrow_length * std::sin(pose.yaw));
  arrow->color(color).line_width(1.5F);
}

void DrawCase(const matplot::figure_handle& figure, size_t index,
              const avp::test::CollisionTestCase& test_case) {
  const matplot::axes_handle axis = matplot::subplot(figure, 2, 3, index + 1);
  const bool collision = avp::IsVehicleObstacleCollision(
      test_case.vehicle_pose, test_case.vehicle, test_case.obstacle_pose,
      test_case.obstacle_length_m, test_case.obstacle_width_m);
  const std::string obstacle_color = collision ? "r" : "g";

  DrawRectangle(axis, test_case.vehicle_pose, test_case.vehicle.length_m, test_case.vehicle.width_m,
                "b-", 1.5F);
  DrawRectangle(axis, test_case.vehicle_pose,
                test_case.vehicle.length_m + 2.0 * test_case.vehicle.safety_margin_m,
                test_case.vehicle.width_m + 2.0 * test_case.vehicle.safety_margin_m, "b--", 1.0F);
  DrawRectangle(axis, test_case.obstacle_pose, test_case.obstacle_length_m,
                test_case.obstacle_width_m, obstacle_color + "-", 1.5F);
  axis->plot(std::vector<double>{test_case.vehicle_pose.position.x},
             std::vector<double>{test_case.vehicle_pose.position.y}, "bo");
  axis->plot(std::vector<double>{test_case.obstacle_pose.position.x},
             std::vector<double>{test_case.obstacle_pose.position.y}, obstacle_color + "o");
  DrawHeading(axis, test_case.vehicle_pose, test_case.vehicle.length_m, "b");
  DrawHeading(axis, test_case.obstacle_pose, test_case.obstacle_length_m, obstacle_color);

  axis->xlim({-3.0, 7.0});
  axis->ylim({-5.0, 5.0});
  matplot::axis(axis, matplot::equal);
  axis->grid(true);
  axis->xlabel("x (m)");
  axis->ylabel("y (m)");
  axis->title(std::string(test_case.name) + " | expected=" +
              (test_case.expected_collision ? "collision" : "separated") + " | actual=" +
              (collision ? "collision" : "separated"));
}

}  // namespace

int main() {
  const matplot::figure_handle figure = matplot::figure(true);
  figure->size(1400, 850);
  for (size_t index = 0; index < avp::test::CollisionTestCases().size(); ++index) {
    DrawCase(figure, index, avp::test::CollisionTestCases()[index]);
  }
  figure->show();
}
