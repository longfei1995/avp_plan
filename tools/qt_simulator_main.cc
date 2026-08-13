#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

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
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPolygonF>
#include <QPushButton>
#include <QScrollArea>
#include <QScreen>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWidget>

#include "tools/plot_data.h"
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
    setFocusPolicy(Qt::StrongFocus);
    Clear();
  }

  void Clear() {
    scene_.clear();
    content_bounds_ = {};
    scene_.setSceneRect(-5.0, -10.0, 25.0, 20.0);
  }

  int SelectedObstacle() const {
    const QList<QGraphicsItem*> selected = scene_.selectedItems();
    if (selected.empty()) return -1;
    return selected.front()->data(kObstacleIndexRole).toInt();
  }

  void FitContents() {
    if (content_bounds_.isValid() && !content_bounds_.isEmpty()) {
      fitInView(content_bounds_, Qt::KeepAspectRatio);
    }
  }

  void Render(const SimulationRuntime& runtime, bool show_debug, bool show_predictions) {
    scene_.clear();
    const SimulationScenario& scenario = runtime.scenario();
    bool has_bounds = false;
    QRectF bounds;
    const auto include_point = [&bounds, &has_bounds](const Vec2& point) {
      const QPointF scene_point = Point(point);
      if (!has_bounds) {
        bounds = QRectF(scene_point, QSizeF());
        has_bounds = true;
        return;
      }
      bounds.setLeft(std::min(bounds.left(), scene_point.x()));
      bounds.setRight(std::max(bounds.right(), scene_point.x()));
      bounds.setTop(std::min(bounds.top(), scene_point.y()));
      bounds.setBottom(std::max(bounds.bottom(), scene_point.y()));
    };

    for (const Lane& lane : scenario.map.lanes) {
      if (lane.centerline.size() < 2) continue;
      QPainterPath path(Point(lane.centerline.front()));
      include_point(lane.centerline.front());
      for (size_t index = 1; index < lane.centerline.size(); ++index) {
        path.lineTo(Point(lane.centerline[index]));
        include_point(lane.centerline[index]);
      }
      scene_.addPath(path, QPen(lane.closed ? Qt::darkGray : QColor("#576b7a"), 0.18,
                                lane.closed ? Qt::DashLine : Qt::SolidLine));
      AddLabel(QString::fromStdString(lane.id), lane.centerline.front());
    }

    for (const ParkingSpot& spot : scenario.map.parking_spots) {
      include_point(spot.entry_pose.position);
      include_point(spot.target_pose.position);
      scene_.addEllipse(Point(spot.entry_pose.position).x() - 0.25,
                        Point(spot.entry_pose.position).y() - 0.25, 0.5, 0.5,
                        QPen(Qt::darkMagenta), QBrush(Qt::magenta));
      scene_.addPolygon(
          VehiclePolygon(spot.target_pose, scenario.vehicle.length_m, scenario.vehicle.width_m),
          QPen(Qt::darkGreen), QBrush(Qt::NoBrush));
      AddLabel(QString::fromStdString(spot.id), spot.target_pose.position);
    }

    if (show_debug) {
      const PlanningDebugData& debug = runtime.debug();
      const auto add_line = [this](const auto& points, const QPen& pen) {
        if (points.size() < 2) return;
        QPainterPath path(Point(points.front().position));
        for (size_t index = 1; index < points.size(); ++index) {
          path.lineTo(Point(points[index].position));
        }
        scene_.addPath(path, pen);
      };
      if (debug.global_route.reference_line.size() >= 2) {
        QPainterPath path(Point(debug.global_route.reference_line.front()));
        for (size_t index = 1; index < debug.global_route.reference_line.size(); ++index) {
          path.lineTo(Point(debug.global_route.reference_line[index]));
        }
        scene_.addPath(path, QPen(QColor("#fa8c16"), 0.13, Qt::DashLine));
      }
      if (!debug.coupling_iterations.empty()) {
        add_line(debug.coupling_iterations.back().local_path, QPen(QColor("#13c2c2"), 0.16));
      }
      for (const ParkingSegment& segment : debug.parking_maneuver.segments) {
        if (segment.points.size() < 2) continue;
        QPainterPath path(Point(segment.points.front().pose.position));
        for (size_t index = 1; index < segment.points.size(); ++index) {
          path.lineTo(Point(segment.points[index].pose.position));
        }
        scene_.addPath(path, QPen(QColor("#722ed1"), 0.12, Qt::DotLine));
      }
    }

    if (!runtime.response().trajectory.empty()) {
      for (const TimedTrajectoryPoint& point : runtime.response().trajectory) {
        include_point(point.pose.position);
      }
      QPainterPath path(Point(runtime.response().trajectory.front().pose.position));
      for (const TimedTrajectoryPoint& point : runtime.response().trajectory) {
        path.lineTo(Point(point.pose.position));
      }
      scene_.addPath(path,
                     QPen(runtime.response().status == PlanningStatus::kOk ? QColor("#1890ff")
                                                                           : Qt::red,
                          0.2));
    }

    for (size_t obstacle_index = 0; obstacle_index < scenario.obstacles.size();
         ++obstacle_index) {
      const ScenarioObstacle& obstacle = scenario.obstacles[obstacle_index];
      const Pose2d pose = SampleObstaclePose(obstacle, runtime.simulation_time_s());
      include_point(pose.position);
      QGraphicsPolygonItem* item = scene_.addPolygon(
          VehiclePolygon(pose, obstacle.length_m, obstacle.width_m), QPen(Qt::red),
          QBrush(QColor("#ffccc7")));
      MakeObstacleSelectable(item, static_cast<int>(obstacle_index));
      QGraphicsSimpleTextItem* label = AddLabel(QString::fromStdString(obstacle.id), pose.position);
      MakeObstacleSelectable(label, static_cast<int>(obstacle_index));
      if (show_predictions) {
        QPainterPath path(Point(pose.position));
        for (double time_s = scenario.planner.time_step_s;
             time_s <= scenario.planner.horizon_s; time_s += scenario.planner.time_step_s) {
          const Vec2 prediction =
              SampleObstaclePose(obstacle, runtime.simulation_time_s() + time_s).position;
          path.lineTo(Point(prediction));
          include_point(prediction);
        }
        scene_.addPath(path, QPen(QColor("#ff7875"), 0.08, Qt::DashLine));
      }
    }

    scene_.addPolygon(
        VehiclePolygon(runtime.ego().pose, scenario.vehicle.length_m, scenario.vehicle.width_m),
        QPen(Qt::blue), QBrush(QColor("#91d5ff")));
    include_point(runtime.ego().pose.position);
    if (has_bounds) {
      const double padding =
          std::max({2.0, scenario.vehicle.length_m * 0.6, scenario.vehicle.width_m});
      content_bounds_ = bounds.adjusted(-padding, -padding, padding, padding);
      scene_.setSceneRect(content_bounds_);
    }
  }

 protected:
  void keyPressEvent(QKeyEvent* event) override {
    if (event->key() == Qt::Key_Delete) {
      emit DeleteObstaclePressed();
      event->accept();
      return;
    }
    QGraphicsView::keyPressEvent(event);
  }

  void wheelEvent(QWheelEvent* event) override {
    const double scale_factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
    scale(scale_factor, scale_factor);
  }

 signals:
  void DeleteObstaclePressed();

 private:
  static constexpr int kObstacleIndexRole = 1;

  static void MakeObstacleSelectable(QGraphicsItem* item, int index) {
    item->setFlag(QGraphicsItem::ItemIsSelectable);
    item->setData(kObstacleIndexRole, index);
  }

  QGraphicsSimpleTextItem* AddLabel(const QString& text, const Vec2& position) {
    QGraphicsSimpleTextItem* label = scene_.addSimpleText(text);
    label->setPos(Point(position));
    label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    return label;
  }

  QGraphicsScene scene_;
  QRectF content_bounds_;
};

class PlotWidget final : public QWidget {
 public:
  PlotWidget(QString title, QString x_label, QString y_label, QWidget* parent = nullptr)
      : QWidget(parent),
        title_(std::move(title)),
        x_label_(std::move(x_label)),
        y_label_(std::move(y_label)) {
    setMinimumHeight(190);
  }

  void SetSeries(std::vector<PlotPoint> points, QString empty_message = "No data",
                 bool step_line = false) {
    points_ = std::move(points);
    empty_message_ = std::move(empty_message);
    step_line_ = step_line;
    update();
  }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), palette().base());
    painter.setPen(palette().text().color());
    painter.drawText(QRectF(0.0, 4.0, width(), 20.0), Qt::AlignCenter, title_);

    const QRectF plot_rect(54.0, 28.0, std::max(1, width() - 72),
                           std::max(1, height() - 66));
    painter.setPen(QPen(QColor("#bfbfbf"), 1.0));
    painter.drawRect(plot_rect);
    painter.drawText(QRectF(plot_rect.left(), plot_rect.bottom() + 22.0, plot_rect.width(), 18.0),
                     Qt::AlignCenter, x_label_);
    painter.save();
    painter.translate(14.0, plot_rect.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-plot_rect.height() / 2.0, -10.0, plot_rect.height(), 18.0),
                     Qt::AlignCenter, y_label_);
    painter.restore();

    if (points_.empty()) {
      painter.setPen(QColor("#8c8c8c"));
      painter.drawText(plot_rect, Qt::AlignCenter, empty_message_);
      return;
    }

    double minimum_x = points_.front().x;
    double maximum_x = points_.front().x;
    double minimum_y = points_.front().y;
    double maximum_y = points_.front().y;
    for (const PlotPoint& point : points_) {
      minimum_x = std::min(minimum_x, point.x);
      maximum_x = std::max(maximum_x, point.x);
      minimum_y = std::min(minimum_y, point.y);
      maximum_y = std::max(maximum_y, point.y);
    }
    ExpandRange(&minimum_x, &maximum_x);
    ExpandRange(&minimum_y, &maximum_y);
    const auto map_point = [&](const PlotPoint& point) {
      const double x = plot_rect.left() +
                       (point.x - minimum_x) / (maximum_x - minimum_x) * plot_rect.width();
      const double y = plot_rect.bottom() -
                       (point.y - minimum_y) / (maximum_y - minimum_y) * plot_rect.height();
      return QPointF(x, y);
    };

    constexpr int kGridCount = 4;
    painter.setPen(QPen(QColor("#e8e8e8"), 1.0, Qt::DashLine));
    for (int index = 1; index < kGridCount; ++index) {
      const double ratio = static_cast<double>(index) / kGridCount;
      painter.drawLine(QPointF(plot_rect.left() + ratio * plot_rect.width(), plot_rect.top()),
                       QPointF(plot_rect.left() + ratio * plot_rect.width(), plot_rect.bottom()));
      painter.drawLine(QPointF(plot_rect.left(), plot_rect.top() + ratio * plot_rect.height()),
                       QPointF(plot_rect.right(), plot_rect.top() + ratio * plot_rect.height()));
    }

    painter.setPen(QColor("#595959"));
    painter.drawText(QRectF(plot_rect.left() - 20.0, plot_rect.bottom() + 2.0, 50.0, 18.0),
                     Qt::AlignLeft, QString::number(minimum_x, 'g', 3));
    painter.drawText(QRectF(plot_rect.right() - 35.0, plot_rect.bottom() + 2.0, 50.0, 18.0),
                     Qt::AlignLeft, QString::number(maximum_x, 'g', 3));
    painter.drawText(QRectF(2.0, plot_rect.top() - 8.0, 48.0, 18.0), Qt::AlignRight,
                     QString::number(maximum_y, 'g', 3));
    painter.drawText(QRectF(2.0, plot_rect.bottom() - 9.0, 48.0, 18.0), Qt::AlignRight,
                     QString::number(minimum_y, 'g', 3));

    QPainterPath path(map_point(points_.front()));
    for (size_t index = 1; index < points_.size(); ++index) {
      const QPointF mapped = map_point(points_[index]);
      if (step_line_) path.lineTo(mapped.x(), path.currentPosition().y());
      path.lineTo(mapped);
    }
    painter.setPen(QPen(QColor("#1677ff"), 1.8));
    painter.drawPath(path);
  }

 private:
  static void ExpandRange(double* minimum, double* maximum) {
    const double span = *maximum - *minimum;
    if (span < 1e-9) {
      const double padding = std::max(1.0, std::abs(*minimum) * 0.1);
      *minimum -= padding;
      *maximum += padding;
      return;
    }
    const double padding = span * 0.08;
    *minimum -= padding;
    *maximum += padding;
  }

  QString title_;
  QString x_label_;
  QString y_label_;
  QString empty_message_ = "No data";
  std::vector<PlotPoint> points_;
  bool step_line_ = false;
};

class MainWindow final : public QMainWindow {
 public:
  MainWindow() {
    setWindowTitle("AVP Qt Planning Simulator");
    const QRect available = QApplication::primaryScreen()->availableGeometry();
    resize(std::min(1400, std::max(900, available.width() - 40)),
           std::min(850, std::max(600, available.height() - 40)));
    CreateUi();
    connect(&timer_, &QTimer::timeout, this, [this] {
      if (runtime_ != nullptr && runtime_->running()) {
        runtime_->Step();
        Refresh();
      }
    });
    timer_.start(20);
    Refresh();
  }

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override {
    if (watched == obstacles_ && event->type() == QEvent::KeyPress &&
        static_cast<QKeyEvent*>(event)->key() == Qt::Key_Delete) {
      DeleteObstacle(obstacles_->currentRow());
      return true;
    }
    return QMainWindow::eventFilter(watched, event);
  }

 private:
  void CreateUi() {
    QToolBar* toolbar = addToolBar("Simulation");
    run_action_ = toolbar->addAction("Run");
    run_action_->setCheckable(true);
    connect(run_action_, &QAction::toggled, this, [this](bool running) {
      if (runtime_ != nullptr) runtime_->SetRunning(running);
    });
    step_action_ = toolbar->addAction("Step");
    connect(step_action_, &QAction::triggered, this, [this] {
      if (runtime_ != nullptr) runtime_->Step();
      Refresh();
    });
    replan_action_ = toolbar->addAction("Replan");
    connect(replan_action_, &QAction::triggered, this, [this] {
      if (runtime_ != nullptr) runtime_->Replan();
      Refresh();
    });
    reset_action_ = toolbar->addAction("Reset");
    connect(reset_action_, &QAction::triggered, this, [this] { ResetScenario(); });
    toolbar->addSeparator();
    connect(toolbar->addAction("Load map"), &QAction::triggered, this,
            [this] { OpenScenario(); });
    save_action_ = toolbar->addAction("Save scenario");
    connect(save_action_, &QAction::triggered, this, [this] { SaveScenario(); });
    delete_obstacle_action_ = toolbar->addAction("Delete obstacle");
    connect(delete_obstacle_action_, &QAction::triggered, this,
            [this] { DeleteSelectedObstacle(); });
    connect(toolbar->addAction("Fit view"), &QAction::triggered, this,
            [this] { canvas_->FitContents(); });

    QSplitter* splitter = new QSplitter(this);
    canvas_ = new Canvas(splitter);
    canvas_->setMinimumWidth(500);
    tabs_ = new QTabWidget(splitter);
    tabs_->setMinimumWidth(380);
    tabs_->setMaximumWidth(560);
    CreateScenarioTab();
    CreatePlotsTab();
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);
    splitter->setSizes({900, 460});
    splitter->setCollapsible(0, false);
    splitter->setCollapsible(1, false);
    setCentralWidget(splitter);
    connect(canvas_, &Canvas::DeleteObstaclePressed, this,
            [this] { DeleteSelectedObstacle(); });
  }

  void CreateScenarioTab() {
    QScrollArea* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scenario_editor_ = new QWidget;
    scroll->setWidget(scenario_editor_);
    QVBoxLayout* layout = new QVBoxLayout(scenario_editor_);
    QFormLayout* form = new QFormLayout;
    ego_x_ = Spin(-1000.0, 1000.0, 0.0);
    ego_y_ = Spin(-1000.0, 1000.0, 0.0);
    ego_yaw_ = Spin(-6.3, 6.3, 0.0);
    target_ = new QComboBox;
    form->addRow("Ego x", ego_x_);
    form->addRow("Ego y", ego_y_);
    form->addRow("Ego yaw", ego_yaw_);
    form->addRow("Target", target_);
    layout->addLayout(form);
    connect(ego_x_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this] { UpdateInitialEgo(); });
    connect(ego_y_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this] { UpdateInitialEgo(); });
    connect(ego_yaw_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this] { UpdateInitialEgo(); });
    connect(target_, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this] { UpdateTarget(); });

    layout->addWidget(new QLabel("Obstacles (keyframes)"));
    obstacles_ = new QListWidget;
    obstacles_->setMaximumHeight(100);
    obstacles_->installEventFilter(this);
    layout->addWidget(obstacles_);
    connect(obstacles_, &QListWidget::currentRowChanged, this,
            [this] { LoadObstacleProperties(); });
    obstacle_length_ = Spin(0.05, 100.0, 0.6);
    obstacle_width_ = Spin(0.05, 100.0, 0.6);
    obstacle_loop_ = new QCheckBox("Loop trajectory");
    obstacle_length_->setPrefix("length ");
    obstacle_width_->setPrefix("width ");
    layout->addWidget(obstacle_length_);
    layout->addWidget(obstacle_width_);
    layout->addWidget(obstacle_loop_);
    QPushButton* apply_obstacle = new QPushButton("Apply obstacle properties");
    layout->addWidget(apply_obstacle);
    connect(apply_obstacle, &QPushButton::clicked, this,
            [this] { ApplyObstacleProperties(); });
    QPushButton* add_obstacle = new QPushButton("Add obstacle");
    layout->addWidget(add_obstacle);
    connect(add_obstacle, &QPushButton::clicked, this, [this] { AddObstacle(); });

    keyframes_ = new QTableWidget(0, 5);
    keyframes_->setHorizontalHeaderLabels({"t", "x", "y", "yaw", "v"});
    keyframes_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    keyframes_->setMinimumHeight(145);
    layout->addWidget(keyframes_);
    QPushButton* add_keyframe = new QPushButton("Add keyframe");
    layout->addWidget(add_keyframe);
    connect(add_keyframe, &QPushButton::clicked, this, [this] { AddKeyframe(); });
    connect(keyframes_, &QTableWidget::itemChanged, this,
            [this](QTableWidgetItem*) { ApplyKeyframeTable(); });

    show_debug_ = new QCheckBox("Show planning stages");
    show_debug_->setChecked(true);
    show_prediction_ = new QCheckBox("Show obstacle predictions");
    show_prediction_->setChecked(true);
    layout->addWidget(show_debug_);
    layout->addWidget(show_prediction_);
    connect(show_debug_, &QCheckBox::toggled, this, [this] { Refresh(); });
    connect(show_prediction_, &QCheckBox::toggled, this, [this] { Refresh(); });
    diagnostics_ = new QPlainTextEdit;
    diagnostics_->setReadOnly(true);
    diagnostics_->setMinimumHeight(150);
    layout->addWidget(diagnostics_);
    layout->addStretch();
    tabs_->addTab(scroll, "Scenario");
  }

  void CreatePlotsTab() {
    QScrollArea* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    QWidget* contents = new QWidget;
    scroll->setWidget(contents);
    QVBoxLayout* layout = new QVBoxLayout(contents);
    layout->addWidget(new QLabel("Current planning"));
    sl_plot_ = AddPlot(layout, "S-L path", "s (m)", "l (m)");
    st_plot_ = AddPlot(layout, "S-T path", "time (s)", "s (m)");
    curvature_plot_ = AddPlot(layout, "S-curvature", "s (m)", "curvature (1/m)");
    path_yaw_plot_ = AddPlot(layout, "S-yaw", "s (m)", "yaw (deg)");
    layout->addWidget(new QLabel("Actual ego history (last 60 s)"));
    speed_plot_ = AddPlot(layout, "Ego speed", "simulation time (s)", "speed (m/s)");
    acceleration_plot_ =
        AddPlot(layout, "Ego acceleration", "simulation time (s)", "acceleration (m/s^2)");
    ego_yaw_plot_ = AddPlot(layout, "Ego yaw", "simulation time (s)", "yaw (deg)");
    gear_plot_ = AddPlot(layout, "Ego gear", "simulation time (s)", "R=-1, N=0, D=1");
    layout->addStretch();
    tabs_->addTab(scroll, "Plots");
  }

  static PlotWidget* AddPlot(QVBoxLayout* layout, const QString& title,
                             const QString& x_label, const QString& y_label) {
    PlotWidget* plot = new PlotWidget(title, x_label, y_label);
    layout->addWidget(plot);
    return plot;
  }

  QDoubleSpinBox* Spin(double minimum, double maximum, double value) {
    QDoubleSpinBox* spin = new QDoubleSpinBox;
    spin->setRange(minimum, maximum);
    spin->setDecimals(3);
    spin->setValue(value);
    return spin;
  }

  void SetLoadedState(bool loaded) {
    run_action_->setEnabled(loaded);
    step_action_->setEnabled(loaded);
    replan_action_->setEnabled(loaded);
    reset_action_->setEnabled(loaded);
    save_action_->setEnabled(loaded);
    delete_obstacle_action_->setEnabled(loaded);
    scenario_editor_->setEnabled(loaded);
  }

  void UpdateInitialEgo() {
    if (runtime_ == nullptr) return;
    SimulationScenario scenario = runtime_->scenario();
    scenario.initial_ego.pose = {{ego_x_->value(), ego_y_->value()}, ego_yaw_->value()};
    ResetAndReplan(std::move(scenario));
    QTimer::singleShot(0, this, [this] { canvas_->FitContents(); });
  }

  void UpdateTarget() {
    if (runtime_ == nullptr || target_->currentIndex() < 0) return;
    SimulationScenario scenario = runtime_->scenario();
    scenario.target_parking_spot_id = target_->currentData().toString().toStdString();
    ResetAndReplan(std::move(scenario));
  }

  void AddObstacle() {
    if (runtime_ == nullptr) return;
    SimulationScenario scenario = runtime_->scenario();
    const int index = static_cast<int>(scenario.obstacles.size() + 1);
    scenario.obstacles.push_back({"obstacle_" + std::to_string(index), 0.6, 0.6, 1.0,
                                  false, {{0.0, {{4.0, 2.0}, 0.0}, 0.0}}});
    ResetAndReplan(std::move(scenario));
  }

  void AddKeyframe() {
    if (runtime_ == nullptr) return;
    const int index = obstacles_->currentRow();
    if (index < 0) return;
    SimulationScenario scenario = runtime_->scenario();
    auto& keyframes = scenario.obstacles[static_cast<size_t>(index)].keyframes;
    const ObstacleKeyframe previous = keyframes.empty() ? ObstacleKeyframe{} : keyframes.back();
    keyframes.push_back({previous.time_s + 1.0, previous.pose, previous.speed_mps});
    ResetAndReplan(std::move(scenario));
  }

  void DeleteSelectedObstacle() {
    int index = canvas_->SelectedObstacle();
    if (index < 0) index = obstacles_->currentRow();
    DeleteObstacle(index);
  }

  void DeleteObstacle(int index) {
    if (runtime_ == nullptr) return;
    SimulationScenario scenario = runtime_->scenario();
    if (index < 0 || index >= static_cast<int>(scenario.obstacles.size())) return;
    scenario.obstacles.erase(scenario.obstacles.begin() + index);
    ResetAndReplan(std::move(scenario));
  }

  void ResetAndReplan(SimulationScenario scenario) {
    runtime_->Reset(std::move(scenario));
    runtime_->Replan();
    Refresh();
  }

  void ResetScenario() {
    if (runtime_ == nullptr) return;
    ResetAndReplan(runtime_->scenario());
  }

  void ApplyObstacleProperties() {
    if (runtime_ == nullptr) return;
    const int index = obstacles_->currentRow();
    if (index < 0) return;
    SimulationScenario scenario = runtime_->scenario();
    ScenarioObstacle& obstacle = scenario.obstacles[static_cast<size_t>(index)];
    obstacle.length_m = obstacle_length_->value();
    obstacle.width_m = obstacle_width_->value();
    obstacle.loop = obstacle_loop_->isChecked();
    ResetAndReplan(std::move(scenario));
  }

  void ApplyKeyframeTable() {
    if (runtime_ == nullptr) return;
    const int index = obstacles_->currentRow();
    if (index < 0 || keyframes_->rowCount() == 0) return;
    SimulationScenario scenario = runtime_->scenario();
    auto& frames = scenario.obstacles[static_cast<size_t>(index)].keyframes;
    if (static_cast<int>(frames.size()) != keyframes_->rowCount()) return;
    for (int row = 0; row < keyframes_->rowCount(); ++row) {
      bool valid = true;
      const auto value = [this, row, &valid](int column) {
        bool parsed = false;
        const double result = keyframes_->item(row, column)->text().toDouble(&parsed);
        valid = valid && parsed && std::isfinite(result);
        return result;
      };
      frames[static_cast<size_t>(row)] = {value(0), {{value(1), value(2)}, value(3)}, value(4)};
      if (!valid) return;
    }
    std::sort(frames.begin(), frames.end(),
              [](const ObstacleKeyframe& left, const ObstacleKeyframe& right) {
                return left.time_s < right.time_s;
              });
    ResetAndReplan(std::move(scenario));
  }

  void RefreshKeyframes() {
    keyframes_->blockSignals(true);
    keyframes_->setRowCount(0);
    if (runtime_ == nullptr || obstacles_->currentRow() < 0) {
      keyframes_->blockSignals(false);
      return;
    }
    const auto& frames =
        runtime_->scenario().obstacles[static_cast<size_t>(obstacles_->currentRow())].keyframes;
    keyframes_->setRowCount(static_cast<int>(frames.size()));
    for (size_t row = 0; row < frames.size(); ++row) {
      const ObstacleKeyframe& frame = frames[row];
      const QStringList values{QString::number(frame.time_s),
                               QString::number(frame.pose.position.x),
                               QString::number(frame.pose.position.y),
                               QString::number(frame.pose.yaw),
                               QString::number(frame.speed_mps)};
      for (int column = 0; column < values.size(); ++column) {
        keyframes_->setItem(static_cast<int>(row), column,
                            new QTableWidgetItem(values[column]));
      }
    }
    keyframes_->blockSignals(false);
  }

  void LoadObstacleProperties() {
    RefreshKeyframes();
    if (runtime_ == nullptr || obstacles_->currentRow() < 0) return;
    const ScenarioObstacle& obstacle =
        runtime_->scenario().obstacles[static_cast<size_t>(obstacles_->currentRow())];
    obstacle_length_->setValue(obstacle.length_m);
    obstacle_width_->setValue(obstacle.width_m);
    obstacle_loop_->setChecked(obstacle.loop);
  }

  void Refresh() {
    SetLoadedState(runtime_ != nullptr);
    if (runtime_ == nullptr) {
      canvas_->Clear();
      diagnostics_->setPlainText("Load a scenario JSON with the Load map action.");
      UpdateEmptyPlots();
      run_action_->blockSignals(true);
      run_action_->setChecked(false);
      run_action_->blockSignals(false);
      return;
    }

    const SimulationScenario& scenario = runtime_->scenario();
    target_->blockSignals(true);
    target_->clear();
    for (const ParkingSpot& spot : scenario.map.parking_spots) {
      target_->addItem(QString::fromStdString(spot.id), QString::fromStdString(spot.id));
    }
    target_->setCurrentIndex(
        target_->findData(QString::fromStdString(scenario.target_parking_spot_id)));
    target_->blockSignals(false);

    ego_x_->blockSignals(true);
    ego_y_->blockSignals(true);
    ego_yaw_->blockSignals(true);
    ego_x_->setValue(scenario.initial_ego.pose.position.x);
    ego_y_->setValue(scenario.initial_ego.pose.position.y);
    ego_yaw_->setValue(scenario.initial_ego.pose.yaw);
    ego_x_->blockSignals(false);
    ego_y_->blockSignals(false);
    ego_yaw_->blockSignals(false);

    obstacles_->blockSignals(true);
    const int selected = obstacles_->currentRow();
    obstacles_->clear();
    for (const ScenarioObstacle& obstacle : scenario.obstacles) {
      obstacles_->addItem(QString::fromStdString(obstacle.id));
    }
    obstacles_->setCurrentRow(std::min(selected, obstacles_->count() - 1));
    obstacles_->blockSignals(false);
    LoadObstacleProperties();

    QString info = QString("t=%1 s\nplan=%2 ms\nstatus=%3\nmode=%4\nmessage=%5\n")
                       .arg(runtime_->simulation_time_s(), 0, 'f', 2)
                       .arg(runtime_->last_planning_time_ms(), 0, 'f', 2)
                       .arg(ToString(runtime_->response().status))
                       .arg(QString::fromStdString(runtime_->debug().planning_mode))
                       .arg(QString::fromStdString(runtime_->response().message));
    for (const std::string& diagnostic : runtime_->response().diagnostics) {
      info += QString::fromStdString(diagnostic) + '\n';
    }
    if (!runtime_->stop_reason().empty()) {
      info += "STOP: " + QString::fromStdString(runtime_->stop_reason());
    }
    diagnostics_->setPlainText(info);
    canvas_->Render(*runtime_, show_debug_->isChecked(), show_prediction_->isChecked());
    UpdatePlots();
    run_action_->blockSignals(true);
    run_action_->setChecked(runtime_->running());
    run_action_->blockSignals(false);
  }

  void UpdateEmptyPlots() {
    for (PlotWidget* plot : {sl_plot_, st_plot_, curvature_plot_, path_yaw_plot_, speed_plot_,
                             acceleration_plot_, ego_yaw_plot_, gear_plot_}) {
      plot->SetSeries({}, "Load a scenario to display data");
    }
  }

  void UpdatePlots() {
    const PlanningPlotData planning =
        BuildPlanningPlotData(runtime_->debug(), runtime_->response());
    sl_plot_->SetSeries(planning.sl_path, QString::fromStdString(planning.sl_empty_message));
    st_plot_->SetSeries(planning.st_path);
    curvature_plot_->SetSeries(planning.curvature_by_s);
    path_yaw_plot_->SetSeries(planning.yaw_by_s);

    std::vector<PlotPoint> speed;
    std::vector<PlotPoint> acceleration;
    std::vector<PlotPoint> yaw;
    std::vector<PlotPoint> gear;
    speed.reserve(runtime_->ego_history().size());
    acceleration.reserve(runtime_->ego_history().size());
    yaw.reserve(runtime_->ego_history().size());
    gear.reserve(runtime_->ego_history().size());
    double unwrapped_yaw = 0.0;
    double previous_yaw = 0.0;
    size_t index = 0;
    for (const EgoHistorySample& sample : runtime_->ego_history()) {
      speed.push_back({sample.time_s, sample.ego.speed_mps});
      acceleration.push_back({sample.time_s, sample.ego.acceleration_mps2});
      if (index == 0) {
        unwrapped_yaw = sample.ego.pose.yaw;
      } else {
        unwrapped_yaw += NormalizeAngle(sample.ego.pose.yaw - previous_yaw);
      }
      previous_yaw = sample.ego.pose.yaw;
      yaw.push_back({sample.time_s, unwrapped_yaw * 180.0 / kPi});
      const double gear_value = sample.ego.direction == DrivingDirection::kDrive
                                    ? 1.0
                                    : (sample.ego.direction == DrivingDirection::kReverse ? -1.0
                                                                                          : 0.0);
      gear.push_back({sample.time_s, gear_value});
      ++index;
    }
    speed_plot_->SetSeries(std::move(speed));
    acceleration_plot_->SetSeries(std::move(acceleration));
    ego_yaw_plot_->SetSeries(std::move(yaw));
    gear_plot_->SetSeries(std::move(gear), "No ego history", true);
  }

  void OpenScenario() {
    const QString path =
        QFileDialog::getOpenFileName(this, "Load map and scenario", {}, "AVP scenario (*.json)");
    if (path.isEmpty()) return;
    SimulationScenario scenario;
    std::string error;
    if (!LoadScenarioJson(path.toStdString(), &scenario, &error)) {
      QMessageBox::warning(this, "Load failed", QString::fromStdString(error));
      return;
    }
    runtime_ = std::make_unique<SimulationRuntime>(std::move(scenario));
    runtime_->Replan();
    Refresh();
    QTimer::singleShot(0, this, [this] { canvas_->FitContents(); });
  }

  void SaveScenario() {
    if (runtime_ == nullptr) return;
    const QString path = QFileDialog::getSaveFileName(this, "Save scenario", "scenario.json",
                                                      "AVP scenario (*.json)");
    if (path.isEmpty()) return;
    std::string error;
    if (!SaveScenarioJson(runtime_->scenario(), path.toStdString(), &error)) {
      QMessageBox::warning(this, "Save failed", QString::fromStdString(error));
    }
  }

  std::unique_ptr<SimulationRuntime> runtime_;
  QTimer timer_;
  QAction* run_action_ = nullptr;
  QAction* step_action_ = nullptr;
  QAction* replan_action_ = nullptr;
  QAction* reset_action_ = nullptr;
  QAction* save_action_ = nullptr;
  QAction* delete_obstacle_action_ = nullptr;
  Canvas* canvas_ = nullptr;
  QTabWidget* tabs_ = nullptr;
  QWidget* scenario_editor_ = nullptr;
  QDoubleSpinBox* ego_x_ = nullptr;
  QDoubleSpinBox* ego_y_ = nullptr;
  QDoubleSpinBox* ego_yaw_ = nullptr;
  QComboBox* target_ = nullptr;
  QListWidget* obstacles_ = nullptr;
  QDoubleSpinBox* obstacle_length_ = nullptr;
  QDoubleSpinBox* obstacle_width_ = nullptr;
  QCheckBox* obstacle_loop_ = nullptr;
  QTableWidget* keyframes_ = nullptr;
  QCheckBox* show_debug_ = nullptr;
  QCheckBox* show_prediction_ = nullptr;
  QPlainTextEdit* diagnostics_ = nullptr;
  PlotWidget* sl_plot_ = nullptr;
  PlotWidget* st_plot_ = nullptr;
  PlotWidget* curvature_plot_ = nullptr;
  PlotWidget* path_yaw_plot_ = nullptr;
  PlotWidget* speed_plot_ = nullptr;
  PlotWidget* acceleration_plot_ = nullptr;
  PlotWidget* ego_yaw_plot_ = nullptr;
  PlotWidget* gear_plot_ = nullptr;
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
