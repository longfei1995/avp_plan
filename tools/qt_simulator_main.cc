#include <cmath>

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPolygonF>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

#include "tools/simulation_runtime.h"

namespace avp::tools {
namespace {

QPointF Point(const Vec2& point) { return {point.x, -point.y}; }

QPolygonF VehiclePolygon(const Pose2d& pose, double length, double width) {
  const Vec2 heading = HeadingAxis(pose.yaw);
  const Vec2 lateral = LateralAxis(pose.yaw);
  QPolygonF polygon;
  for (const std::pair<double, double>& corner :
       {std::pair{0.5, 0.5}, {0.5, -0.5}, {-0.5, -0.5}, {-0.5, 0.5}}) {
    polygon << Point({pose.position.x + corner.first * length * heading.x +
                          corner.second * width * lateral.x,
                      pose.position.y + corner.first * length * heading.y +
                          corner.second * width * lateral.y});
  }
  return polygon;
}

class Canvas final : public QGraphicsView {
  Q_OBJECT

 public:
  explicit Canvas(QWidget* parent = nullptr) : QGraphicsView(parent) {
    setScene(&scene_);
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
    scene_.setSceneRect(-25.0, -25.0, 80.0, 50.0);
  }

  void SetLaneEdit(bool enabled) {
    lane_edit_ = enabled;
    setDragMode(enabled ? QGraphicsView::NoDrag : QGraphicsView::ScrollHandDrag);
  }
  bool lane_edit() const { return lane_edit_; }
  std::vector<Vec2> TakeDrawnLane() {
    std::vector<Vec2> result = std::move(drawn_lane_);
    drawn_lane_.clear();
    return result;
  }
  void Render(const SimulationRuntime& runtime, bool show_debug, bool show_predictions) {
    scene_.clear();
    const SimulationScenario& scenario = runtime.scenario();
    for (const Lane& lane : scenario.map.lanes) {
      if (lane.centerline.size() < 2) continue;
      QPainterPath path(Point(lane.centerline.front()));
      for (size_t index = 1; index < lane.centerline.size(); ++index) path.lineTo(Point(lane.centerline[index]));
      scene_.addPath(path, QPen(lane.closed ? Qt::darkGray : QColor("#576b7a"), 0.18,
                                 lane.closed ? Qt::DashLine : Qt::SolidLine));
      scene_.addSimpleText(QString::fromStdString(lane.id))->setPos(Point(lane.centerline.front()));
    }
    for (const ParkingSpot& spot : scenario.map.parking_spots) {
      scene_.addEllipse(Point(spot.entry_pose.position).x() - .25, Point(spot.entry_pose.position).y() - .25,
                         .5, .5, QPen(Qt::darkMagenta), QBrush(Qt::magenta));
      scene_.addPolygon(VehiclePolygon(spot.target_pose, scenario.vehicle.length_m,
                                       scenario.vehicle.width_m),
                        QPen(Qt::darkGreen), QBrush(Qt::NoBrush));
      scene_.addSimpleText(QString::fromStdString(spot.id))->setPos(Point(spot.target_pose.position));
    }
    if (show_debug) {
      const PlanningDebugData& debug = runtime.debug();
      auto add_line = [this](const auto& points, const QPen& pen) {
        if (points.size() < 2) return;
        QPainterPath path(Point(points.front().position));
        for (size_t index = 1; index < points.size(); ++index) path.lineTo(Point(points[index].position));
        scene_.addPath(path, pen);
      };
      if (debug.global_route.reference_line.size() >= 2) {
        QPainterPath path(Point(debug.global_route.reference_line.front()));
        for (size_t i = 1; i < debug.global_route.reference_line.size(); ++i) path.lineTo(Point(debug.global_route.reference_line[i]));
        scene_.addPath(path, QPen(QColor("#fa8c16"), .13, Qt::DashLine));
      }
      if (!debug.coupling_iterations.empty()) add_line(debug.coupling_iterations.back().local_path, QPen(QColor("#13c2c2"), .16));
      for (const ParkingSegment& segment : debug.parking_maneuver.segments) {
        if (segment.points.size() < 2) continue;
        QPainterPath path(Point(segment.points.front().pose.position));
        for (size_t i = 1; i < segment.points.size(); ++i) path.lineTo(Point(segment.points[i].pose.position));
        scene_.addPath(path, QPen(QColor("#722ed1"), .12, Qt::DotLine));
      }
    }
    if (!runtime.response().trajectory.empty()) {
      QPainterPath path(Point(runtime.response().trajectory.front().pose.position));
      for (const TimedTrajectoryPoint& point : runtime.response().trajectory) path.lineTo(Point(point.pose.position));
      scene_.addPath(path, QPen(runtime.response().status == PlanningStatus::kOk ? QColor("#1890ff") : Qt::red, .2));
    }
    for (const ScenarioObstacle& obstacle : scenario.obstacles) {
      const Pose2d pose = SampleObstaclePose(obstacle, runtime.simulation_time_s());
      scene_.addPolygon(VehiclePolygon(pose, obstacle.length_m, obstacle.width_m), QPen(Qt::red), QBrush(QColor("#ffccc7")));
      scene_.addSimpleText(QString::fromStdString(obstacle.id))->setPos(Point(pose.position));
      if (show_predictions) {
        QPainterPath path(Point(pose.position));
        for (double time = scenario.planner.time_step_s; time <= scenario.planner.horizon_s; time += scenario.planner.time_step_s) {
          path.lineTo(Point(SampleObstaclePose(obstacle, runtime.simulation_time_s() + time).position));
        }
        scene_.addPath(path, QPen(QColor("#ff7875"), .08, Qt::DashLine));
      }
    }
    scene_.addPolygon(VehiclePolygon(runtime.ego().pose, scenario.vehicle.length_m, scenario.vehicle.width_m),
                      QPen(Qt::blue), QBrush(QColor("#91d5ff")));
    if (lane_edit_ && !drawn_lane_.empty()) {
      QPainterPath path(Point(drawn_lane_.front()));
      for (size_t i = 1; i < drawn_lane_.size(); ++i) path.lineTo(Point(drawn_lane_[i]));
      scene_.addPath(path, QPen(Qt::black, .15, Qt::DashLine));
    }
  }

 protected:
  void mousePressEvent(QMouseEvent* event) override {
    if (lane_edit_ && event->button() == Qt::LeftButton) {
      const QPointF point = mapToScene(event->pos());
      drawn_lane_.push_back({point.x(), -point.y()});
      emit LaneChanged();
      event->accept();
      return;
    }
    QGraphicsView::mousePressEvent(event);
  }
  void wheelEvent(QWheelEvent* event) override {
    const double scale_factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
    scale(scale_factor, scale_factor);
  }

 signals:
  void LaneChanged();

 private:
  QGraphicsScene scene_;
  bool lane_edit_ = false;
  std::vector<Vec2> drawn_lane_;
};

class MainWindow final : public QMainWindow {
 public:
  MainWindow() : runtime_(MakeDefaultScenario()) {
    setWindowTitle("AVP Qt Planning Simulator");
    resize(1300, 800);
    CreateUi();
    connect(&timer_, &QTimer::timeout, this, [this] {
      if (runtime_.running()) runtime_.Step();
      Refresh();
    });
    timer_.start(20);
    runtime_.Replan();
    Refresh();
  }

 private:
  void CreateUi() {
    auto* toolbar = addToolBar("Simulation");
    auto* run = toolbar->addAction("Run");
    run->setCheckable(true);
    connect(run, &QAction::toggled, this, [this](bool running) { runtime_.SetRunning(running); });
    connect(toolbar->addAction("Step"), &QAction::triggered, this, [this] { runtime_.Step(); Refresh(); });
    connect(toolbar->addAction("Replan"), &QAction::triggered, this, [this] { runtime_.Replan(); Refresh(); });
    connect(toolbar->addAction("Reset"), &QAction::triggered, this, [this] { runtime_.Reset(runtime_.scenario()); runtime_.Replan(); Refresh(); });
    connect(toolbar->addAction("Open"), &QAction::triggered, this, [this] { OpenScenario(); });
    connect(toolbar->addAction("Save"), &QAction::triggered, this, [this] { SaveScenario(); });
    auto* edit_lane = toolbar->addAction("Draw lane");
    edit_lane->setCheckable(true);
    connect(edit_lane, &QAction::toggled, this, [this](bool enabled) { canvas_->SetLaneEdit(enabled); Refresh(); });
    connect(toolbar->addAction("Commit lane"), &QAction::triggered, this, [this] { CommitLane(); });

    auto* splitter = new QSplitter(this);
    canvas_ = new Canvas(splitter);
    auto* panel = new QWidget(splitter);
    auto* layout = new QVBoxLayout(panel);
    auto* form = new QFormLayout;
    ego_x_ = Spin(-1000, 1000, runtime_.ego().pose.position.x);
    ego_y_ = Spin(-1000, 1000, runtime_.ego().pose.position.y);
    ego_yaw_ = Spin(-6.3, 6.3, runtime_.ego().pose.yaw);
    form->addRow("Ego x", ego_x_); form->addRow("Ego y", ego_y_); form->addRow("Ego yaw", ego_yaw_);
    target_ = new QComboBox; form->addRow("Target", target_);
    layout->addLayout(form);
    connect(ego_x_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { UpdateInitialEgo(); });
    connect(ego_y_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { UpdateInitialEgo(); });
    connect(ego_yaw_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { UpdateInitialEgo(); });
    connect(target_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { UpdateTarget(); });
    layout->addWidget(new QLabel("Selected parking spot"));
    spot_entry_x_ = Spin(-1000, 1000, 0.0); spot_entry_y_ = Spin(-1000, 1000, 0.0);
    spot_target_x_ = Spin(-1000, 1000, 0.0); spot_target_y_ = Spin(-1000, 1000, 0.0);
    spot_target_yaw_ = Spin(-6.3, 6.3, 0.0);
    layout->addWidget(spot_entry_x_); spot_entry_x_->setPrefix("entry x ");
    layout->addWidget(spot_entry_y_); spot_entry_y_->setPrefix("entry y ");
    layout->addWidget(spot_target_x_); spot_target_x_->setPrefix("target x ");
    layout->addWidget(spot_target_y_); spot_target_y_->setPrefix("target y ");
    layout->addWidget(spot_target_yaw_); spot_target_yaw_->setPrefix("target yaw ");
    auto* apply_spot = new QPushButton("Apply parking spot"); layout->addWidget(apply_spot);
    connect(apply_spot, &QPushButton::clicked, this, [this] { ApplyParkingSpot(); });
    layout->addWidget(new QLabel("Lane properties"));
    lanes_ = new QListWidget; layout->addWidget(lanes_);
    lane_id_ = new QLineEdit; lane_successors_ = new QLineEdit; lane_closed_ = new QCheckBox("Closed");
    layout->addWidget(lane_id_); lane_id_->setPlaceholderText("Lane ID");
    layout->addWidget(lane_successors_); lane_successors_->setPlaceholderText("Successor IDs, comma-separated");
    layout->addWidget(lane_closed_);
    auto* apply_lane = new QPushButton("Apply lane properties"); layout->addWidget(apply_lane);
    connect(lanes_, &QListWidget::currentRowChanged, this, [this] { LoadLaneProperties(); });
    connect(apply_lane, &QPushButton::clicked, this, [this] { ApplyLaneProperties(); });
    layout->addWidget(new QLabel("Obstacles (keyframes)"));
    obstacles_ = new QListWidget; layout->addWidget(obstacles_);
    connect(obstacles_, &QListWidget::currentRowChanged, this, [this] { RefreshKeyframes(); });
    obstacle_length_ = Spin(.05, 100.0, .6); obstacle_width_ = Spin(.05, 100.0, .6);
    obstacle_loop_ = new QCheckBox("Loop trajectory");
    layout->addWidget(obstacle_length_); obstacle_length_->setPrefix("length ");
    layout->addWidget(obstacle_width_); obstacle_width_->setPrefix("width ");
    layout->addWidget(obstacle_loop_);
    auto* apply_obstacle = new QPushButton("Apply obstacle properties"); layout->addWidget(apply_obstacle);
    connect(apply_obstacle, &QPushButton::clicked, this, [this] { ApplyObstacleProperties(); });
    auto* add_obstacle = new QPushButton("Add obstacle"); layout->addWidget(add_obstacle);
    connect(add_obstacle, &QPushButton::clicked, this, [this] { AddObstacle(); });
    keyframes_ = new QTableWidget(0, 5); keyframes_->setHorizontalHeaderLabels({"t", "x", "y", "yaw", "v"});
    layout->addWidget(keyframes_);
    auto* add_keyframe = new QPushButton("Add keyframe"); layout->addWidget(add_keyframe);
    connect(add_keyframe, &QPushButton::clicked, this, [this] { AddKeyframe(); });
    connect(keyframes_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) { ApplyKeyframeTable(); });
    show_debug_ = new QCheckBox("Show planning stages"); show_debug_->setChecked(true); layout->addWidget(show_debug_);
    show_prediction_ = new QCheckBox("Show obstacle predictions"); show_prediction_->setChecked(true); layout->addWidget(show_prediction_);
    diagnostics_ = new QPlainTextEdit; diagnostics_->setReadOnly(true); layout->addWidget(diagnostics_, 1);
    setCentralWidget(splitter);
    connect(canvas_, &Canvas::LaneChanged, this, [this] { Refresh(); });
  }
  QDoubleSpinBox* Spin(double minimum, double maximum, double value) {
    auto* spin = new QDoubleSpinBox; spin->setRange(minimum, maximum); spin->setDecimals(3); spin->setValue(value); return spin;
  }
  void UpdateInitialEgo() {
    SimulationScenario scenario = runtime_.scenario();
    scenario.initial_ego.pose = {{ego_x_->value(), ego_y_->value()}, ego_yaw_->value()};
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void UpdateTarget() {
    if (target_->currentIndex() < 0) return;
    SimulationScenario scenario = runtime_.scenario();
    scenario.target_parking_spot_id = target_->currentData().toString().toStdString();
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void ApplyParkingSpot() {
    const int index = target_->currentIndex();
    if (index < 0) return;
    SimulationScenario scenario = runtime_.scenario();
    ParkingSpot& spot = scenario.map.parking_spots[static_cast<size_t>(index)];
    spot.entry_pose.position = {spot_entry_x_->value(), spot_entry_y_->value()};
    spot.target_pose = {{spot_target_x_->value(), spot_target_y_->value()}, spot_target_yaw_->value()};
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void CommitLane() {
    std::vector<Vec2> points = canvas_->TakeDrawnLane();
    if (points.size() < 2) return;
    SimulationScenario scenario = runtime_.scenario();
    const std::string id = "lane_" + std::to_string(scenario.map.lanes.size() + 1);
    scenario.map.lanes.push_back({id, std::move(points), {}, false});
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void AddObstacle() {
    SimulationScenario scenario = runtime_.scenario();
    const int index = static_cast<int>(scenario.obstacles.size() + 1);
    scenario.obstacles.push_back({"obstacle_" + std::to_string(index), .6, .6, 1.0, false,
                                  {{0.0, {{4.0, 2.0}, 0.0}, 0.0}}});
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void AddKeyframe() {
    const int index = obstacles_->currentRow();
    if (index < 0) return;
    SimulationScenario scenario = runtime_.scenario();
    auto& keyframes = scenario.obstacles[static_cast<size_t>(index)].keyframes;
    const ObstacleKeyframe previous = keyframes.back();
    keyframes.push_back({previous.time_s + 1.0, previous.pose, previous.speed_mps});
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void LoadLaneProperties() {
    const int index = lanes_->currentRow();
    if (index < 0) return;
    const Lane& lane = runtime_.scenario().map.lanes[static_cast<size_t>(index)];
    lane_id_->setText(QString::fromStdString(lane.id));
    QStringList successors;
    for (const std::string& id : lane.successor_ids) successors << QString::fromStdString(id);
    lane_successors_->setText(successors.join(',')); lane_closed_->setChecked(lane.closed);
  }
  void ApplyLaneProperties() {
    const int index = lanes_->currentRow();
    if (index < 0 || lane_id_->text().trimmed().isEmpty()) return;
    SimulationScenario scenario = runtime_.scenario();
    Lane& lane = scenario.map.lanes[static_cast<size_t>(index)];
    lane.id = lane_id_->text().trimmed().toStdString(); lane.closed = lane_closed_->isChecked();
    lane.successor_ids.clear();
    for (const QString& id : lane_successors_->text().split(',', Qt::SkipEmptyParts)) {
      lane.successor_ids.push_back(id.trimmed().toStdString());
    }
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void ApplyObstacleProperties() {
    const int index = obstacles_->currentRow();
    if (index < 0) return;
    SimulationScenario scenario = runtime_.scenario();
    ScenarioObstacle& obstacle = scenario.obstacles[static_cast<size_t>(index)];
    obstacle.length_m = obstacle_length_->value(); obstacle.width_m = obstacle_width_->value();
    obstacle.loop = obstacle_loop_->isChecked();
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void ApplyKeyframeTable() {
    const int index = obstacles_->currentRow();
    if (index < 0 || keyframes_->rowCount() == 0) return;
    SimulationScenario scenario = runtime_.scenario();
    auto& frames = scenario.obstacles[static_cast<size_t>(index)].keyframes;
    if (static_cast<int>(frames.size()) != keyframes_->rowCount()) return;
    for (int row = 0; row < keyframes_->rowCount(); ++row) {
      bool valid = true;
      const auto value = [this, row, &valid](int column) {
        bool parsed = false; const double result = keyframes_->item(row, column)->text().toDouble(&parsed);
        valid = valid && parsed && std::isfinite(result); return result;
      };
      frames[static_cast<size_t>(row)] = {value(0), {{value(1), value(2)}, value(3)}, value(4)};
      if (!valid) return;
    }
    std::sort(frames.begin(), frames.end(), [](const ObstacleKeyframe& left, const ObstacleKeyframe& right) {
      return left.time_s < right.time_s;
    });
    runtime_.Reset(std::move(scenario)); runtime_.Replan();
  }
  void RefreshKeyframes() {
    keyframes_->blockSignals(true);
    keyframes_->setRowCount(0);
    const int index = obstacles_->currentRow();
    if (index < 0) return;
    const auto& frames = runtime_.scenario().obstacles[static_cast<size_t>(index)].keyframes;
    keyframes_->setRowCount(static_cast<int>(frames.size()));
    for (size_t row = 0; row < frames.size(); ++row) {
      const auto& frame = frames[row];
      const QStringList values{QString::number(frame.time_s),
                               QString::number(frame.pose.position.x),
                               QString::number(frame.pose.position.y),
                               QString::number(frame.pose.yaw),
                               QString::number(frame.speed_mps)};
      for (int column = 0; column < values.size(); ++column) {
        keyframes_->setItem(static_cast<int>(row), column, new QTableWidgetItem(values[column]));
      }
    }
    keyframes_->blockSignals(false);
  }
  void Refresh() {
    const SimulationScenario& scenario = runtime_.scenario();
    target_->blockSignals(true); target_->clear();
    for (const ParkingSpot& spot : scenario.map.parking_spots) target_->addItem(QString::fromStdString(spot.id), QString::fromStdString(spot.id));
    target_->setCurrentIndex(target_->findData(QString::fromStdString(scenario.target_parking_spot_id))); target_->blockSignals(false);
    if (target_->currentIndex() >= 0) {
      const ParkingSpot& spot = scenario.map.parking_spots[static_cast<size_t>(target_->currentIndex())];
      spot_entry_x_->setValue(spot.entry_pose.position.x); spot_entry_y_->setValue(spot.entry_pose.position.y);
      spot_target_x_->setValue(spot.target_pose.position.x); spot_target_y_->setValue(spot.target_pose.position.y);
      spot_target_yaw_->setValue(spot.target_pose.yaw);
    }
    ego_x_->blockSignals(true); ego_y_->blockSignals(true); ego_yaw_->blockSignals(true);
    ego_x_->setValue(scenario.initial_ego.pose.position.x); ego_y_->setValue(scenario.initial_ego.pose.position.y);
    ego_yaw_->setValue(scenario.initial_ego.pose.yaw);
    ego_x_->blockSignals(false); ego_y_->blockSignals(false); ego_yaw_->blockSignals(false);
    lanes_->blockSignals(true); const int lane_selected = lanes_->currentRow(); lanes_->clear();
    for (const Lane& lane : scenario.map.lanes) lanes_->addItem(QString::fromStdString(lane.id));
    lanes_->setCurrentRow(std::min(lane_selected, lanes_->count() - 1)); lanes_->blockSignals(false); LoadLaneProperties();
    obstacles_->blockSignals(true); const int selected = obstacles_->currentRow(); obstacles_->clear();
    for (const ScenarioObstacle& obstacle : scenario.obstacles) obstacles_->addItem(QString::fromStdString(obstacle.id));
    obstacles_->setCurrentRow(std::min(selected, obstacles_->count() - 1)); obstacles_->blockSignals(false);
    if (obstacles_->currentRow() >= 0) {
      const ScenarioObstacle& obstacle = scenario.obstacles[static_cast<size_t>(obstacles_->currentRow())];
      obstacle_length_->setValue(obstacle.length_m); obstacle_width_->setValue(obstacle.width_m);
      obstacle_loop_->setChecked(obstacle.loop);
    }
    RefreshKeyframes();
    QString info = QString("t=%1 s\nplan=%2 ms\nstatus=%3\nmode=%4\nmessage=%5\n")
        .arg(runtime_.simulation_time_s(), 0, 'f', 2)
        .arg(runtime_.last_planning_time_ms(), 0, 'f', 2)
        .arg(ToString(runtime_.response().status))
        .arg(QString::fromStdString(runtime_.debug().planning_mode))
        .arg(QString::fromStdString(runtime_.response().message));
    for (const std::string& diagnostic : runtime_.response().diagnostics) info += QString::fromStdString(diagnostic) + '\n';
    if (!runtime_.stop_reason().empty()) info += "STOP: " + QString::fromStdString(runtime_.stop_reason());
    diagnostics_->setPlainText(info);
    canvas_->Render(runtime_, show_debug_->isChecked(), show_prediction_->isChecked());
  }
  void OpenScenario() {
    const QString path = QFileDialog::getOpenFileName(this, "Open scenario", {}, "Scenario (*.json)");
    if (path.isEmpty()) return;
    SimulationScenario scenario; std::string error;
    if (!LoadScenarioJson(path.toStdString(), &scenario, &error)) { QMessageBox::warning(this, "Open failed", QString::fromStdString(error)); return; }
    runtime_.Reset(std::move(scenario)); runtime_.Replan(); Refresh();
  }
  void SaveScenario() {
    const QString path = QFileDialog::getSaveFileName(this, "Save scenario", "scenario.json", "Scenario (*.json)");
    if (path.isEmpty()) return;
    std::string error;
    if (!SaveScenarioJson(runtime_.scenario(), path.toStdString(), &error)) QMessageBox::warning(this, "Save failed", QString::fromStdString(error));
  }

  SimulationRuntime runtime_;
  QTimer timer_;
  Canvas* canvas_ = nullptr;
  QDoubleSpinBox* ego_x_ = nullptr;
  QDoubleSpinBox* ego_y_ = nullptr;
  QDoubleSpinBox* ego_yaw_ = nullptr;
  QComboBox* target_ = nullptr;
  QDoubleSpinBox* spot_entry_x_ = nullptr;
  QDoubleSpinBox* spot_entry_y_ = nullptr;
  QDoubleSpinBox* spot_target_x_ = nullptr;
  QDoubleSpinBox* spot_target_y_ = nullptr;
  QDoubleSpinBox* spot_target_yaw_ = nullptr;
  QListWidget* lanes_ = nullptr;
  QLineEdit* lane_id_ = nullptr;
  QLineEdit* lane_successors_ = nullptr;
  QCheckBox* lane_closed_ = nullptr;
  QListWidget* obstacles_ = nullptr;
  QDoubleSpinBox* obstacle_length_ = nullptr;
  QDoubleSpinBox* obstacle_width_ = nullptr;
  QCheckBox* obstacle_loop_ = nullptr;
  QTableWidget* keyframes_ = nullptr;
  QCheckBox* show_debug_ = nullptr;
  QCheckBox* show_prediction_ = nullptr;
  QPlainTextEdit* diagnostics_ = nullptr;
};
}  // namespace
}  // namespace avp::tools

int main(int argc, char* argv[]) {
  QApplication application(argc, argv);
  avp::tools::MainWindow window;
  window.show();
  return application.exec();
}

#include "qt_simulator_main.moc"
