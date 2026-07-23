#ifndef AVP_INTERFACES_ADAPTERS_H_
#define AVP_INTERFACES_ADAPTERS_H_

#include <string>

#include "common/types.h"

namespace avp {

class LocalizationAdapter {
 public:
  bool Adapt(const PlanningRequest& request, PlanningFrame* frame, std::string* error) const;
};
class PerceptionAdapter {
 public:
  bool Adapt(const PlanningRequest& request, PlanningFrame* frame, std::string* error) const;
};
class MapAdapter {
 public:
  bool Adapt(const PlanningRequest& request, PlanningFrame* frame, std::string* error) const;
};
class TaskAdapter {
 public:
  bool Adapt(const PlanningRequest& request, PlanningFrame* frame, std::string* error) const;
};

// 适配器是将协议特定数据转换为 PlanningFrame 的唯一位置。
class PlanningFrameAdapter {
 public:
  bool Adapt(const PlanningRequest& request, const VehicleConfig& vehicle,
             const PlannerConfig& config, PlanningFrame* frame, std::string* error) const;

 private:
  LocalizationAdapter localization_adapter_;
  PerceptionAdapter perception_adapter_;
  MapAdapter map_adapter_;
  TaskAdapter task_adapter_;
};
}  // avp 命名空间
#endif  // 头文件保护
