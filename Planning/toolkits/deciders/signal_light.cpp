
/**
 * @file
 **/

#include "../../toolkits/deciders/signal_light.h"
#include <limits>
#include <vector>
#include "../../common/ego_info.h"
#include "../../common/frame.h"
#include "../../toolkits/deciders/utill.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {

using common::Status;
using perception::TrafficLight;
using perception::TrafficLightDetection;

SignalLight::SignalLight(const TrafficRuleConfig& config)
    : TrafficRule(config) {}

Status SignalLight::ApplyRule(Frame* const frame,
                              ReferenceLineInfo* const reference_line_info)
{
  // 1.estimate current reference line contain traffic post or not
  if (!FindValidSignalLight(reference_line_info)) {
    return Status::OK;
  }
  cout<<"Note car is driving on crossroad, please observing traffic light status !"<<endl;
  // 2.read signal
  ReadSignals();
  // 3.
  MakeDecisions(frame, reference_line_info);
  return Status::OK;
}

bool SignalLight::FindValidSignalLight(ReferenceLineInfo* const reference_line_info)
{
  auto traffic_lights =
        reference_line_info->reference_line().MapRoute().SignalLightOverlaps();

  if (!traffic_lights.has_traffic_light) {
    cout<< "No signal lights from reference line."<<endl;
    return false;
  }

   // estimate signal overlap invalid ? if car beyond the segment start_s 3.0m not add
  if (traffic_lights.start_s+
      config_.one_of_config.signal_light.min_pass_s_distance >
      reference_line_info->AdcSlBoundary().end_s &&
      traffic_lights.start_s - reference_line_info->AdcSlBoundary().end_s <
      g_config_param.speed_upper_bound * g_config_param.trajectory_time_length )
    {
      traffic_lights_from_route_ = traffic_lights;
    }


  return traffic_lights_from_route_.ids.size() > 0;
}

void SignalLight::ReadSignals() {
  // 1. estimate get traffic light or not ?
  detected_signals_.clear();
  if (!AdapterManager::GetTrafficLightDetection()) {
    return;
  }
  // 2. estimate data timeliness
  if (AdapterManager::GetTrafficLightDetectionDelaySec()>
      config_.one_of_config.signal_light.signal_expire_time_sec) {//5.0 s
      cout<< "traffic signals msg is expired: "
          << AdapterManager::GetTrafficLightDetectionDelaySec();
    return;
  }
  // 3. get traffic signal data
  const TrafficLightDetection& detection =
        AdapterManager::GetTrafficLightDetectionLatestObserved();
  for (int j = 0; j < detection.traffic_light.size(); j++)
   {
     const TrafficLight& signal = detection.traffic_light.at(j);
     cout<<"Note camera has detected signal light and id is : "<<signal.id<<endl;
     detected_signals_[signal.id] = signal;
   }
}

void SignalLight::MakeDecisions(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info)
{

  cout<<"SignalLight::MakeDecisions adc_front_s = "
      <<reference_line_info->AdcSlBoundary().end_s<<endl;
  cout<<"SignalLight::MakeDecisions adc_speed = "
      <<reference_line_info->GetVehicleState().linear_velocity<<endl;

for (auto id : traffic_lights_from_route_.ids)
  {
    const TrafficLight signal = GetSignal(id);
    double stop_deceleration  = GetADCStopDeceleration(
              reference_line_info, traffic_lights_from_route_.start_s +
              config_.one_of_config.signal_light.stop_distance ,
              config_.one_of_config.signal_light.min_pass_s_distance );//8.0 m
    cout<<"stop_deceleration = "<<stop_deceleration<<endl<<endl;

    if (((signal.color == perception::RED &&
         stop_deceleration < config_.one_of_config.signal_light.max_stop_deceleration) ||
        (signal.color == perception::Unknown &&
         stop_deceleration < config_.one_of_config.signal_light.max_stop_deceleration) ||
        (signal.color == perception::YELLOW &&
         stop_deceleration <
         config_.one_of_config.signal_light.max_stop_deacceleration_yellow_light))&&
        (signal.confidence > 0))
    {

      if (BuildStopDecision(frame, reference_line_info, &traffic_lights_from_route_)) {
        cout<<"signal light Build Stop Decision!!!"<<endl;
      }
    }

  }
}



void SignalLight::SetCreepForwardSignalDecision(
    ReferenceLineInfo* const reference_line_info,
    hdmap::TrafficLightInfo* const signal_light) const {
  //CHECK_NOTNULL(signal_light);

  if (EgoInfo::instance()->start_point().v >
      config_.one_of_config.signal_light.righ_turn_creep.speed_limit) {
    cout<< "Do not creep forward due to large speed."<<endl;
    return;
  }

  const double creep_s_buffer =
      config_.one_of_config.signal_light.righ_turn_creep.min_boundary_s;
  const auto& path_decision = reference_line_info->path_decision();
  for (const auto& path_obstacle : path_decision->path_obstacles().Items())
  {
    const auto& st_boundary = path_obstacle->reference_line_st_boundary();
    const double stop_s =
        signal_light->start_s - config_.one_of_config.signal_light.stop_distance;
    if (reference_line_info->AdcSlBoundary().end_s + st_boundary.min_s() <
        stop_s + creep_s_buffer) {
      cout << "Do not creep forward because obstacles are close."<<endl;
      return;
    }
  }

  signal_light->start_s = reference_line_info->AdcSlBoundary().end_s +
                          config_.one_of_config.signal_light.stop_distance +
                          creep_s_buffer;
  cout<< "Creep forward s = " << signal_light->start_s<<endl;
}

TrafficLight SignalLight::GetSignal(const int32_t& signal_id) {

  auto  result = detected_signals_.find(signal_id);

  if (result == detected_signals_.end()) {
    cout<<"Warning crossroad light id: "<<signal_id<<" don't match with signal light id"<<endl;
    TrafficLight traffic_light;
    traffic_light.id = (signal_id);
    traffic_light.color = (perception::Unknown);
    traffic_light.confidence = (0.0);
    traffic_light.tracking_time = (0.0);
    return traffic_light;
  }
  cout<<"Note signal light id: "<<signal_id<<"find matching with crossroad light id"<<endl;
  return result->second;
}

bool SignalLight::BuildStopDecision( Frame* const frame,
                                     ReferenceLineInfo*  const reference_line_info,
                                     hdmap::TrafficLight* const traffic_light  ) {
  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), traffic_light->start_s)) {
    cout<< "signal_light " << traffic_light->ids.front()
        << " is not on reference line"<<endl;
    return false;
  }
  // create virtual stop wall obstacle_id is negative
  const double stop_s =
   traffic_light->start_s - config_.one_of_config.signal_light.stop_distance;
  int32_t virtual_obstacle_id = traffic_light->ids.front() * (-1);
  auto* obstacle = frame->CreateStopObstacle(
           reference_line_info, virtual_obstacle_id, stop_s );
  if (!obstacle) {
    cout << "Failed to create obstacle[" << virtual_obstacle_id << "]"<<endl;
    return false;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    cout<< "Failed to create path_obstacle for " << virtual_obstacle_id<<endl;
    return false;
  }
  // build stop decision

  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).theta;

  ObjectDecisionType decision_type_;
  decision_type_.has_stop = true;
  decision_type_.object_tag_case = 7;
  auto* stop_decision = &decision_type_.stop;
  auto signal_color = GetSignal(traffic_light->ids.front()).color;

  if (signal_color == perception::YELLOW) {
    stop_decision->reason_code = StopReasonCode::STOP_REASON_YELLOW_SIGNAL;
  } else {
    stop_decision->reason_code = StopReasonCode::STOP_REASON_SIGNAL;
  }
  stop_decision->distance_s  = stop_s ;

  stop_decision->stop_heading = stop_heading;
  stop_decision->stop_point.x = stop_point.x;
  stop_decision->stop_point.y = stop_point.y;
  stop_decision->stop_point.z = 0.0;

  auto* path_decision = reference_line_info->path_decision();
  if (!path_decision->MergeWithMainStop(decision_type_.stop, stop_wall->Id(),
                                        reference_line_info->reference_line(),
                                        reference_line_info->AdcSlBoundary() ) ) {
    cout<< "signal " << traffic_light->ids.front()
        << "is not the closest stop."<<endl;
    return false;
  }

  path_decision->AddLongitudinalDecision(GetName(config_.rule_id),
                                         stop_wall->Id(), decision_type_);

  return true;
}

}  // namespace planning

