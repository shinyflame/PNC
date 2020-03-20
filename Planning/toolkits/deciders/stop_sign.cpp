
/**
 * @file
 **/

#include "../../toolkits/deciders/stop_sign.h"
#include <algorithm>
#include <limits>
#include <unordered_set>
#include "../../common/get_now_time.h"
#include "../../common/frame.h"
#include "../../toolkits/deciders/utill.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;




namespace planning {
bool   StopSign::is_stop_ = false;
double StopSign::stop_begin_time_ = 0;
int    StopSign::update_station_id_ = -1;

using  common::Status;

StopSign::StopSign(const TrafficRuleConfig& config) : TrafficRule(config) {}

Status StopSign::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  //CHECK_NOTNULL(frame);
  //CHECK_NOTNULL(reference_line_info);
  vehicle_state_ = reference_line_info->GetVehicleState();
  if (!FindBusStation(reference_line_info)) {
    update_station_id_ = -1;//clear have stop station id
    return Status::OK;
  }

  if(update_station_id_ != GetStationId())
   {
     if(!CompleteStop())
         MakeDecisions(frame, reference_line_info);
   }else{
     is_stop_ = false;
     cout<<"Complete stop on station: "<<GetStationId()
         <<", now continue driving !!!"<<endl;
   }

  return Status::OK;
}

bool StopSign::FindBusStation(ReferenceLineInfo* const reference_line_info){

  auto station = reference_line_info->reference_line().MapRoute().GetBusStation();

  if( station.id >0){
      cout<<"route contain station id ="<<station.id<<endl;
      station_id = station.id;
      station_ = station;
      auto adc_sl = reference_line_info->GetVechicleSLPoint();
      if(adc_sl.s > station.start_s && adc_sl.s < station.end_s){
          adc_within_station_ = true;
        }
      return true;
    }

  return false;

}

void StopSign::MakeDecisions(Frame* const frame,
                             ReferenceLineInfo* const reference_line_info){

  BuildStopDecision( frame, reference_line_info, &station_ );

}

bool StopSign::BuildStopDecision( Frame* const frame,
                                  ReferenceLineInfo* const reference_line_info,
                                  hdmap::Station* const station ){
  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length(), station->end_s)) {
    cout<< "bus staiton" << station->id
        << " is not on reference line"<<endl;
    return true;
  }
  // create virtual stop wall obstacle_id is negative
  const double stop_s =
   station->end_s - config_.one_of_config.stop_sign.stop_distance;// 1.0 meter
  int32_t virtual_obstacle_id = station->id * (-1);
  auto* obstacle = frame->CreateStopObstacle(
           reference_line_info, virtual_obstacle_id, stop_s);
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
  stop_decision->reason_code = StopReasonCode::STOP_REASON_STOP_SIGN ;
  stop_decision->distance_s  = stop_s;

  stop_decision->stop_heading = stop_heading;
  stop_decision->stop_point.x = stop_point.x;
  stop_decision->stop_point.y = stop_point.y;
  stop_decision->stop_point.z = 0.0;

  auto* path_decision = reference_line_info->path_decision();
  if (!path_decision->MergeWithMainStop(decision_type_.stop, stop_wall->Id(),
                                        reference_line_info->reference_line(),
                                        reference_line_info->AdcSlBoundary() ) ) {
    cout<< "bus station " << station->id
        << " is not the closest stop."<<endl;
    return false;
  }

  path_decision->AddLongitudinalDecision(GetName(config_.rule_id),
                                         stop_wall->Id(), decision_type_);

  return true;

}
double StopSign::GetBusHaveStopTime(){

  if((vehicle_state_.linear_velocity < 0.2) && (!is_stop_) ){

      stop_begin_time_ = Clock::NowInSeconds();
      is_stop_ = true;
    }

  if(is_stop_)
    return Clock::NowInSeconds() - stop_begin_time_;
  else
    return 0;
 }

bool StopSign::CompleteStop(){

  if(adc_within_station_)
    if( GetBusHaveStopTime() > config_.one_of_config.stop_sign.wait_timeout ){
        update_station_id_  = GetStationId();//remenber have stop station id
        return true;
     }

   return false;

 }


}  // namespace planning

