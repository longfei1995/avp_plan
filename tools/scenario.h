#ifndef AVP_TOOLS_SCENARIO_H_
#define AVP_TOOLS_SCENARIO_H_

#include <string>
#include <vector>

#include "common/types.h"

namespace avp::tools {

struct ObstacleKeyframe {
  double time_s = 0.0;
  Pose2d pose;
  double speed_mps = 0.0;
};

struct ScenarioObstacle {
  std::string id;
  double length_m = 0.6;
  double width_m = 0.6;
  double confidence = 1.0;
  bool loop = false;
  std::vector<ObstacleKeyframe> keyframes;
};

struct SimulationScenario {
  int schema_version = 1;
  MapSnapshot map;
  EgoState initial_ego;
  std::string target_parking_spot_id;
  std::vector<ScenarioObstacle> obstacles;
  VehicleConfig vehicle;
  PlannerConfig planner;
};

SimulationScenario MakeDefaultScenario();
Pose2d SampleObstaclePose(const ScenarioObstacle& obstacle, double time_s);
double SampleObstacleSpeed(const ScenarioObstacle& obstacle, double time_s);
PlanningRequest MakePlanningRequest(const SimulationScenario& scenario, const EgoState& ego,
                                    double simulation_time_s, uint64_t sequence_id);

// JSON is intentionally owned by the tool rather than the planner's production API.
bool SaveScenarioJson(const SimulationScenario& scenario, const std::string& path,
                      std::string* error);
bool LoadScenarioJson(const std::string& path, SimulationScenario* scenario, std::string* error);

}  // namespace avp::tools

#endif  // AVP_TOOLS_SCENARIO_H_
