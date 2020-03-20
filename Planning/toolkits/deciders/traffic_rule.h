
/**
 * @file
 **/

#ifndef MODULES_PLANNING_TOOLKITS_DECIDERS_TRAFFIC_RULE_H_
#define MODULES_PLANNING_TOOLKITS_DECIDERS_TRAFFIC_RULE_H_

#include <string>
#include "../../common/struct.h"
#include "../../common/frame.h"
#include "../../common/reference_line_info.h"

using namespace TrafficRuleStruct;

namespace planning {


class TrafficRule {
 public:
  explicit TrafficRule(const TrafficRuleConfig& config) : config_(config) {}
  virtual ~TrafficRule() = default;
  virtual RuleId Id() const { return config_.rule_id; }
  const TrafficRuleConfig& GetConfig() const { return config_; }
  virtual common::Status ApplyRule(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info) = 0;
  PlanningStatusStruct::PlanningStatus
                         *GetPlanningStatus(){return &planning_status_;}
  std::string GetName(RuleId id) {
    switch (id) {
        case 1: return "BacksideVehicle";  break;
        case 2: return "ChangeLane"; break;
        case 3: return "Crosswalk"; break;
        case 4: return "Destination"; break;
        case 5: return "FrontVehicle"; break;
        case 6: return "KeepClear"; break;
        case 7: return "PullOver"; break;
        case 8: return "ReferenceLineEnd"; break;
        case 9: return "Rerouting"; break;
        case 10:return "SignalLight"; break;
        case 11:return "StopSignConfig"; break;

        default:
        return "Error";
        break;
    }
}
  //perception::ObstacleType
std::string GetObstacleTypeName(perception::ObstacleType id)
  {
    switch (id) {
        case 1: return "UNKNOWN_MOVABLE";  break;
        case 2: return "UNKNOWN_UNMOVABLE"; break;
        case 3: return "PEDESTRIAN "; break;
        case 4: return "BICYCLE"; break;
        case 5: return "VEHICLE"; break;

        default:
        return "UNKNOWN";
        break;
    }
 }
std::string GetRoadStatusName(PlanningStatusStruct::RoadStatus id)
 {
    switch (id) {
        case 1: return "UNKNOWN";  break;
        case 2: return "IN_OPERATION"; break;
        case 3: return "DONE"; break;
        case 4: return "DISABLED"; break;
        case 5: return "DRIVE"; break;
        case 6: return "STOP";  break;
        case 7: return "WAIT"; break;
        case 8: return "SIDEPASS"; break;
        case 9: return "CREEP"; break;
        case 10: return "STOP_DONE"; break;
        case 11: return "IN_CHANGE_LANE"; break;
        case 12: return "CHANGE_LANE_FAILED"; break;
        case 13: return "CHANGE_LANE_SUCCESS"; break;

      default: return "UNKNOWN"; break;
    }
 }


  template <typename T>
  bool WithinBound(T start, T end, T value) {
    return value >= start && value <= end;
  }


 protected:
  TrafficRuleConfig config_;
  PlanningStatusStruct::PlanningStatus planning_status_;
};

}  // namespace planning


#endif  // MODULES_PLANNING_TOOLKITS_DECIDERS_TRAFFIC_RULE_H_
