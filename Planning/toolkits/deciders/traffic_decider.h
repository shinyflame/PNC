
/**
 * @file
 **/

#ifndef PLANNING_TOOLKITS_DECIDERS_TRAFFIC_DECIDER_H_
#define PLANNING_TOOLKITS_DECIDERS_TRAFFIC_DECIDER_H_

#include <string>
#include <vector>

#include "../../common/util/factory.h"
#include "../../reference_line/reference_line.h"
#include "../../toolkits/deciders/traffic_rule.h"
#include "../../common/struct.h"

namespace planning {

/**
 * @class TrafficDecider
 * @brief Create traffic related decision in this class.
 * The created obstacles is added to obstacles_, and the decision is added to
 * path_obstacles_
 * Traffic obstacle examples include:
 *  * Traffic Light
 *  * End of routing
 *  * Select the drivable reference line.
 */
class TrafficDecider {
 public:
  TrafficDecider() = default;
  bool Init(const TrafficRuleConfigs &config);
  virtual ~TrafficDecider() = default;
  common::Status Execute(Frame *frame,
                                 ReferenceLineInfo *reference_line_info);

 private:
  static util::Factory<RuleId, TrafficRule,
         TrafficRule *(*)(const TrafficRuleConfig &config)> s_rule_factory;

  void RegisterRules();
  void BuildPlanningTarget(ReferenceLineInfo *reference_line_info);

  TrafficRuleConfigs rule_configs_;
};

}  // namespace planning


#endif  // MODULES_PLANNING_TOOLKITS_DECIDERS_TRAFFIC_DECIDER_H_
