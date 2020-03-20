
/**
 * @file
 **/

#include "../../toolkits/deciders/traffic_decider.h"
#include <limits>

//#include "toolkits/deciders/backside_vehicle.h"
//#include "toolkits/deciders/change_lane.h"
//#include "modules/planning/toolkits/deciders/creeper.h"
//#include "toolkits/deciders/crosswalk.h"

//#include "modules/planning/toolkits/deciders/front_vehicle.h"
//#include "modules/planning/toolkits/deciders/keep_clear.h"
//#include "modules/planning/toolkits/deciders/pull_over.h"
//#include "modules/planning/toolkits/deciders/reference_line_end.h"
//#include "modules/planning/toolkits/deciders/rerouting.h"
#include "../../toolkits/deciders/destination.h"
#include "../../toolkits/deciders/signal_light.h"
#include "../../toolkits/deciders/stop_sign.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;


namespace planning {
using common::Status;

util::Factory<RuleId, TrafficRule,TrafficRule *(*)(const TrafficRuleConfig &config)>
TrafficDecider::s_rule_factory;

void TrafficDecider::RegisterRules() {

#if 0
  s_rule_factory.Register(BACKSIDE_VEHICLE,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new BacksideVehicle(config);
                          });

  s_rule_factory.Register(CHANGE_LANE,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new ChangeLane(config);
                          });

  s_rule_factory.Register(CROSSWALK,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new Crosswalk(config);
                          });

  s_rule_factory.Register(TrafficRule::FRONT_VEHICLE,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new FrontVehicle(config);
                          });
  s_rule_factory.Register(TrafficRule::KEEP_CLEAR,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new KeepClear(config);
                          });
  s_rule_factory.Register(TrafficRule::PULL_OVER,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new PullOver(config);
                          });
  s_rule_factory.Register(TrafficRule::REFERENCE_LINE_END,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new ReferenceLineEnd(config);
                          });
  s_rule_factory.Register(TrafficRule::REROUTING,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new Rerouting(config);
                        });
# endif
//  static util::Factory<RuleId, TrafficRule, TrafficRule *(*)(const TrafficRuleConfig &config)> s_rule_factory;

  s_rule_factory.Register(DESTINATION,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new Destination(config);
                          });
  s_rule_factory.Register(STOP_SIGN,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new StopSign(config);
                          });

  s_rule_factory.Register(SIGNAL_LIGHT,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new SignalLight(config);
                          });

}

bool TrafficDecider::Init(const TrafficRuleConfigs &config) {
  if (s_rule_factory.Empty()) {
    RegisterRules();
  }
  rule_configs_ = config;
  return true;
}


void TrafficDecider::BuildPlanningTarget(ReferenceLineInfo *reference_line_info) {

  double min_s = std::numeric_limits<double>::infinity();
  StopPoint stop_point;

  for (const auto *obstacle :
            reference_line_info->path_decision()->path_obstacles().Items()) {
    
    if (obstacle->obstacle()->IsVirtual() &&
        obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop &&
        obstacle->PerceptionSLBoundary().start_s < min_s)
     {
      min_s = obstacle->PerceptionSLBoundary().start_s;
      const auto &stop_code =obstacle->LongitudinalDecision().stop.reason_code;

      if (stop_code == StopReasonCode::STOP_REASON_DESTINATION ||
          stop_code == StopReasonCode::STOP_REASON_CROSSWALK ||
          stop_code == StopReasonCode::STOP_REASON_STOP_SIGN ||
          stop_code == StopReasonCode::STOP_REASON_YIELD_SIGN ||
          stop_code == StopReasonCode::STOP_REASON_CREEPER ||
          stop_code == StopReasonCode::STOP_REASON_REFERENCE_END ||
          stop_code == StopReasonCode::STOP_REASON_SIGNAL)
      {
        stop_point.type = StopType::HARD;
        cout << "Hard stop at: " << min_s<<endl;
        cout << "REASON: " << stop_code<<endl;
      } else if (stop_code == StopReasonCode::STOP_REASON_YELLOW_SIGNAL) {
        stop_point.type = StopType::SOFT;
        cout << "Soft stop at: " << min_s << "  STOP_REASON_YELLOW_SIGNAL"<<endl;
      } else {
        cout << "No planning target found at reference line."<<endl;
      }
    }
  }

  if (min_s != std::numeric_limits<double>::infinity()) {
    const auto &vehicle_config = g_vehicle_config;
     //common::VehicleConfigHelper::instance()->GetConfig();
    double front_edge_to_center = vehicle_config.front_edge_to_center;
    stop_point.s=
        (min_s - front_edge_to_center + g_config_param.virtual_stop_wall_length/2.0);
                                                    //virtual_stop_wall_length = 0.1
     reference_line_info->SetStopPoint(stop_point);
  }
}


Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  //CHECK_NOTNULL(frame);
  //CHECK_NOTNULL(reference_line_info);

  for (const auto &rule_config : rule_configs_.config)
  {
    if (!rule_config.enabled) {
      cout << "Rule " << rule_config.rule_id << " not enabled"<<endl;
      continue; }

    auto rule = s_rule_factory.CreateObject(rule_config.rule_id, rule_config);
    if (!rule) {
      cout << "Could not find rule: " << rule_config.rule_id<<endl;
      continue; }

    rule->ApplyRule(frame, reference_line_info);
    cout<< "Applied rule: "<< rule->GetName(rule_config.rule_id)<<endl;
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK;
}

}  // namespace planning

