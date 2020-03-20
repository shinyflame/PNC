
/**
 * @file
 **/

#ifndef PLANNING_TOOLKITS_DECIDERS_SIGNAL_LIGHT_H_
#define PLANNING_TOOLKITS_DECIDERS_SIGNAL_LIGHT_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "../../common/struct.h"
#include "../../toolkits/deciders/traffic_rule.h"


namespace planning {

class SignalLight : public TrafficRule {
 public:
  explicit SignalLight(const TrafficRuleConfig& config);

  virtual ~SignalLight() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void ReadSignals();
  bool FindValidSignalLight(ReferenceLineInfo* const reference_line_info);
   perception::TrafficLight GetSignal(const int32_t& signal_id);
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  bool BuildStopDecision(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info,
                         hdmap::TrafficLight* const traffic_light);
  void SetCreepForwardSignalDecision(
      ReferenceLineInfo* const reference_line_info,
      hdmap::TrafficLightInfo* const signal_light) const;

 private:
  static constexpr int32_t const SIGNAL_LIGHT_VO_ID_PREFIX = -1;
  hdmap::TrafficLight traffic_lights_from_route_;
  std::unordered_map<int32_t, perception::TrafficLight> detected_signals_;
};

}  // namespace planning


#endif  // MODULES_PLANNING_TOOLKITS_DECIDERS_SIGNAL_LIGHT_H_
