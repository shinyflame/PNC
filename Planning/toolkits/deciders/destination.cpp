


#include "../../toolkits/deciders/destination.h"
#include <algorithm>
#include <limits>
#include <unordered_set>
#include "../../common/get_now_time.h"
#include "../../common/frame.h"
#include "../../toolkits/deciders/utill.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;


namespace planning {

using  common::Status;

Destination::Destination(const TrafficRuleConfig& config) : TrafficRule(config) {}

Status Destination::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {

  vehicle_state_ = reference_line_info->GetVehicleState();
  if (!FindDestination(reference_line_info)) {

    return Status::OK;
  }


  MakeDecisions(frame, reference_line_info);

  return Status::OK;
}

bool Destination::FindDestination(ReferenceLineInfo* const reference_line_info){

  auto destination = reference_line_info->reference_line().MapRoute().GetDestination();

  if( destination.id >0){
      cout<<"route contain terminal id ="<<destination.id<<endl;
      destination_ = destination;
      return true;
    }

  return false;

}

void Destination::MakeDecisions( Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info){

  BuildStopDecision( frame, reference_line_info, &destination_);

}

bool Destination::BuildStopDecision( Frame* const frame,
                                     ReferenceLineInfo* const reference_line_info,
                                     hdmap::Terminal* const destination ){
  // check
  const auto& reference_line = reference_line_info->reference_line();
  if (!WithinBound(0.0, reference_line.Length() +
                   config_.one_of_config.destination.stop_distance,
                   destination->start_s)) {
    cout<< " destination s =:" << destination->start_s
        << " destination id =:"<< destination->id
        << " reference_line.Length = "<<reference_line.Length()<<endl;
    return true;
  }
  // create virtual stop wall obstacle_id is negative
  const double stop_s =
   destination->start_s - config_.one_of_config.destination.stop_distance;// 1.0 meter
  int32_t virtual_obstacle_id = destination->id * (-1);
  auto* obstacle = frame->CreateStopObstacle(
           reference_line_info, virtual_obstacle_id, stop_s);
  if (!obstacle) {
    cout << "Failed to create destination virtual obstacle["
         << virtual_obstacle_id << "]"<<endl;
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
  stop_decision->reason_code = StopReasonCode::STOP_REASON_DESTINATION ;
  stop_decision->distance_s  = stop_s;

  stop_decision->stop_heading = stop_heading;
  stop_decision->stop_point.x = stop_point.x;
  stop_decision->stop_point.y = stop_point.y;
  stop_decision->stop_point.z = 0.0;

  auto* path_decision = reference_line_info->path_decision();
  if (!path_decision->MergeWithMainStop(decision_type_.stop, stop_wall->Id(),
                                        reference_line_info->reference_line(),
                                        reference_line_info->AdcSlBoundary() ) ) {
    cout<< "bus station " << destination->id
        << " is not the closest stop."<<endl;
    return false;
  }

  path_decision->AddLongitudinalDecision(GetName(config_.rule_id),
                                         stop_wall->Id(), decision_type_);

  return true;

}



}  // namespace planning

