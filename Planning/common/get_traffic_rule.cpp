
#include "../common/get_traffic_rule.h"

namespace planning {


namespace TrafficRuleStruct {

  bool GetTrafficeRule(TrafficRuleConfigs &Rulers){

    TrafficRuleConfig rule;
#if 0
     rule.rule_id = BACKSIDE_VEHICLE;
     rule.enabled = true;
     rule.one_of_config.backside_vehicle.backside_lane_width = 4.0;
     Rulers.config.push_back(rule);

     rule.rule_id = CHANGE_LANE;
     rule.enabled = true;
     rule.one_of_config.change_lane.min_overtake_distance = 10.0;//m
     rule.one_of_config.change_lane.min_overtake_time     = 2.0;//S
     rule.one_of_config.change_lane.enable_guard_obstacle = true;
     rule.one_of_config.change_lane.guard_distance  = 100.0;// m
     rule.one_of_config.change_lane.min_guard_speed = 1.0;  //  m/s
     Rulers.config.push_back(rule);

     rule.rule_id = CROSSWALK;
     rule.enabled = true;
     rule.one_of_config.crosswalk.stop_distance = 1.0;
     rule.one_of_config.crosswalk.max_stop_deceleration = 6.0;
     rule.one_of_config.crosswalk.min_pass_s_distance = 1.0;
     rule.one_of_config.crosswalk.max_stop_speed = 0.3;
     rule.one_of_config.crosswalk.max_valid_stop_distance =3.5;
     rule.one_of_config.crosswalk.expand_s_distance = 2.0;
     rule.one_of_config.crosswalk.stop_strick_l_distance = 6.0;
     rule.one_of_config.crosswalk.stop_loose_l_distance = 8.0;
     rule.one_of_config.crosswalk.stop_timeout = 4.0;
     Rulers.config.push_back(rule);

     rule.rule_id = FRONT_VEHICLE;
     rule.enabled = true;
     rule.one_of_config.front_vehicle.enable_side_pass = true;
     rule.one_of_config.front_vehicle.side_pass_s_threshold = 15.0;
     rule.one_of_config.front_vehicle.side_pass_l_threshold = 1.0;
     rule.one_of_config.front_vehicle.side_pass_wait_time = 30.0;
     rule.one_of_config.front_vehicle.nudge_l_buffer = 0.5;
     Rulers.config.push_back(rule);

     rule.rule_id = KEEP_CLEAR;
     rule.enabled = true;
     rule.one_of_config.keep_clear.enable_keep_clear_zone = true;
     rule.one_of_config.keep_clear.enable_junction = false;
     rule.one_of_config.keep_clear.min_pass_s_distance = 2.0;
     Rulers.config.push_back(rule);

     rule.rule_id = PULL_OVER;
     rule.enabled = true;
     rule.one_of_config.pull_over.stop_distance = 0.5;
     rule.one_of_config.pull_over.max_stop_speed = 0.2;
     rule.one_of_config.pull_over.max_valid_stop_distance = 3.0;
     rule.one_of_config.pull_over.max_stop_deceleration = 2.5;
     rule.one_of_config.pull_over.min_pass_s_distance = 1.0;
     rule.one_of_config.pull_over.buffer_to_boundary = 0.5;
     rule.one_of_config.pull_over.plan_distance = 55.0;
     rule.one_of_config.pull_over.operation_length = 50.0;
     rule.one_of_config.pull_over.max_check_distance = 60.0;
     rule.one_of_config.pull_over.max_failure_count = 10;
     Rulers.config.push_back(rule);

     rule.rule_id = REFERENCE_LINE_END;
     rule.enabled = true;
     rule.one_of_config.reference_line_end.stop_distance = 0.5;
     rule.one_of_config.reference_line_end.min_reference_line_remain_length = 50.0;
     Rulers.config.push_back(rule);

     rule.rule_id = REROUTING;
     rule.enabled = false;
     rule.one_of_config.rerouting.cooldown_time = 2.0 ;//s
     rule.one_of_config.rerouting.cooldown_time = 3.0 ;//s
     Rulers.config.push_back(rule);

#endif

     rule.rule_id = DESTINATION;
     rule.enabled = true;
     rule.one_of_config.destination.enable_pull_over = true;
     rule.one_of_config.destination.stop_distance = 2;
     rule.one_of_config.destination.pull_over_plan_distance = 5;
     Rulers.config.push_back(rule);


     rule.rule_id = STOP_SIGN;
     rule.enabled = false;
     rule.one_of_config.stop_sign.stop_distance = 1;
     rule.one_of_config.stop_sign.min_pass_s_distance = 3.0;
     rule.one_of_config.stop_sign.max_stop_speed = 0.3;
     rule.one_of_config.stop_sign.max_valid_stop_distance = 3.5;
     rule.one_of_config.stop_sign.stop_duration = 1.0;
     rule.one_of_config.stop_sign.watch_vehicle_max_valid_stop_speed = 0.5;
     rule.one_of_config.stop_sign.watch_vehicle_max_valid_stop_distance = 5.0;
     rule.one_of_config.stop_sign.wait_timeout = 5.0;
     rule.one_of_config.stop_sign.creep.enabled = true;
     rule.one_of_config.stop_sign.creep.creep_distance_to_stop_line = 1.5;
     rule.one_of_config.stop_sign.creep.stop_distance = 0.3;
     rule.one_of_config.stop_sign.creep.speed_limit = 0.4;
     rule.one_of_config.stop_sign.creep.max_valid_stop_distance = 0.4;
     rule.one_of_config.stop_sign.creep.min_boundary_t = 6.0;
     Rulers.config.push_back(rule);

     rule.rule_id = SIGNAL_LIGHT;
     rule.enabled = false;
     rule.one_of_config.signal_light.stop_distance = 1 ;
     rule.one_of_config.signal_light.max_stop_deceleration = 6.0;
     rule.one_of_config.signal_light.min_pass_s_distance = 2.0; //apollo is 4.0 m
     rule.one_of_config.signal_light.max_stop_deacceleration_yellow_light = 5.0;
     rule.one_of_config.signal_light.signal_expire_time_sec = 5.0;
     rule.one_of_config.signal_light.righ_turn_creep.enabled = false;
     rule.one_of_config.signal_light.righ_turn_creep.min_boundary_t = 6.0;
     rule.one_of_config.signal_light.righ_turn_creep.stop_distance = 0.5;
     rule.one_of_config.signal_light.righ_turn_creep.speed_limit = 1.0;
     Rulers.config.push_back(rule);

     return Rulers.config.size()>0;

  }


}//TrafficRule namespace
}
