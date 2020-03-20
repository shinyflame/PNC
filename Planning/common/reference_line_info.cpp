

#include "reference_line_info.h"
#include "../common/get_data.h"
#include "../math/math_utils.h"
#include "../math/path_matcher.h"

extern planning::ConfigParam g_config_param;
extern planning::VehicleParam g_vehicle_config;

namespace planning {

ReferenceLineInfo::ReferenceLineInfo(const VehicleState& vehicle_state,
                        const TrajectoryPoint& adc_planning_point,//planing start point
                        const ReferenceLine& reference_line,
                        const hdmap::RouteSegment& segments )
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),route_(segments)
{
  Vec2d vehicle_position(vehicle_state_.x, vehicle_state_.y);
  if(!reference_line_.XYToSL(vehicle_position,&vehicle_sl_))
    cout << "Failed to get vehicle_sl_ from vehicle_position"<<endl;
  else
    cout<<"vehicle_sl ( "<<vehicle_sl_.s<<" , "<<vehicle_sl_.l<<" )"<<endl;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

const hdmap::RouteSegment& ReferenceLineInfo::Lanes() const
 {
   return  route_;
 }

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles)
{
    const auto& param = g_vehicle_config;
    // stitching point
    const auto& path_point = adc_planning_point_.path_point;
    Vec2d position(path_point.x, path_point.y);
    Vec2d vec_to_center(
        (param.front_edge_to_center - param.back_edge_to_center) / 2.0,
        (param.left_edge_to_center - param.right_edge_to_center) / 2.0  );
    Vec2d center(position + vec_to_center.rotate(path_point.theta));
    Box2d box(center, path_point.theta, param.length, param.width);
    // realtime vehicle position
    Vec2d vehicle_position(vehicle_state_.x, vehicle_state_.y);
    Vec2d vehicle_center(vehicle_position +
                         vec_to_center.rotate(vehicle_state_.heading));
    Box2d vehicle_box(vehicle_center, vehicle_state_.heading, param.length,param.width);


    if (!reference_line_.GetSLBoundary(vehicle_box,
                                       &sl_boundary_info_.vehicle_sl_boundary_)) {
      cout << "Failed to get ADC boundary from vehicle_box(realtime position "
              "of the car): "
           <<vehicle_box.DebugString()<<endl; ;
      return false;
    }
    //get vehicle in part map sl_boundary
    if (!reference_line_.GetSLBoundary(box,
                                       &sl_boundary_info_.adc_sl_boundary_)) {
      cout << "Failed to get ADC boundary from box: "<<box.DebugString()<< endl;
      return false;
    }
    //judge vehicle sl_boundary valid or not ?
//    constexpr double kOutOfReferenceLineS = 10;
//    if ( sl_boundary_info_.adc_sl_boundary_.end_s   >
//         reference_line_.Length() - kOutOfReferenceLineS||
//         sl_boundary_info_.adc_sl_boundary_.end_s   < 0 ||
//         sl_boundary_info_.adc_sl_boundary_.start_s >
//         reference_line_.Length() - kOutOfReferenceLineS ) {
//         cout << "Vehicle SL ("
//              << sl_boundary_info_.adc_sl_boundary_.start_s<<","
//              << sl_boundary_info_.adc_sl_boundary_.end_s  <<")"
//              << " is not on reference line:[0, " << reference_line_.Length()
//              << "]"<<endl;
//        return false;
//    }
//    constexpr double kOutOfReferenceLineL = 3.0;  // in meters
//    if (sl_boundary_info_.adc_sl_boundary_.start_l > kOutOfReferenceLineL ||
//        sl_boundary_info_.adc_sl_boundary_.end_l < -kOutOfReferenceLineL) {
//      cout << "Ego vehicle is too far away from reference line. "
//           << "strat_l = "<<sl_boundary_info_.adc_sl_boundary_.start_l<<" ,"
//           << "end_l = "  <<sl_boundary_info_.adc_sl_boundary_.start_l<<endl;
//      return false;
//    }

    is_on_reference_line_ =
        reference_line_.IsOnLane(sl_boundary_info_.adc_sl_boundary_);

    //set lattice planning target speed limit;
    SetCruiseSpeed(g_config_param.default_cruise_speed);

    is_inited_ = true;
    return true;


}

bool ReferenceLineInfo::IsInited() const { return is_inited_; }

//bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
//  constexpr double kEpsilon = 1e-2;
//  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
//}

#if 0
void ReferenceLineInfo::SetJunctionRightOfWay(double junction_s,bool is_protected)
{
  auto* right_of_way = &AdapterManager::GetPlanningStatus()->right_of_way;
  auto* junction_right_of_way = &right_of_way->junction;

  for (const auto& overlap : reference_line_.map_path().junction_overlaps()) {
    if (WithinOverlap(overlap, junction_s))
    {
      (*junction_right_of_way)[overlap.object_id] = is_protected;
    }
  }
}


RightOfWayProtectedStatus ReferenceLineInfo::GetRightOfWayStatus() const
{
  auto* right_of_way = &AdapterManager::GetPlanningStatus()->right_of_way;
  auto* junction_right_of_way = &right_of_way->junction;
  for (const auto& overlap : reference_line_.map_path().junction_overlaps())
  {
    if (overlap.end_s < sl_boundary_info_.adc_sl_boundary_.start_s)
     {
        junction_right_of_way->erase(overlap.object_id);
     } else if (WithinOverlap(overlap,sl_boundary_info_.adc_sl_boundary_.end_s))
     {
        auto is_protected = (*junction_right_of_way)[overlap.object_id];
        if (is_protected) {
          return PROTECTED;
        }
        else
        {
           const auto lane_segments =
               reference_line_.GetLaneSegments(overlap.start_s, overlap.end_s);
           for (const auto& segment : lane_segments)
           {
             if (segment.lane->lane().turn != map_lane::NO_TURN)
               { return UNPROTECTED;}
           }
           return PROTECTED;
        }
     }
  }
  return UNPROTECTED;
}
#endif
bool ReferenceLineInfo::CheckChangeLane() const {
  if (!IsChangeLanePath()) {
    cout << "Not a change lane path."<<endl;
    return false;
  }
  for (const auto* path_obstacle : path_decision_.path_obstacles().Items()) {
      const auto& sl_boundary = path_obstacle->PerceptionSLBoundary();
      // if the car is out current lane continue next obastacle judge
      constexpr float kLateralShift = 2.5;
      if (sl_boundary.start_l < -kLateralShift ||
          sl_boundary.end_l > kLateralShift) {
        continue;
      }

      constexpr float kSafeTime = 3.0;
      constexpr float kForwardMinSafeDistance = 6.0;
      constexpr float kBackwardMinSafeDistance = 8.0;

      const float kForwardSafeDistance =
          std::max(kForwardMinSafeDistance,
                   static_cast<float>((adc_planning_point_.v -
                   path_obstacle->obstacle()->Speed()) *kSafeTime));

      const float kBackwardSafeDistance =
          std::max(kBackwardMinSafeDistance,
               static_cast<float>((path_obstacle->obstacle()->Speed() -
                                   adc_planning_point_.v) *kSafeTime));

      if (sl_boundary.end_s > sl_boundary_info_.adc_sl_boundary_.start_s -
          kBackwardSafeDistance && sl_boundary.start_s <
          sl_boundary_info_.adc_sl_boundary_.end_s + kForwardSafeDistance)
      {
        return false;
      }
    }
    return true;
  }

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }
//if the car position not on segment the car change lane path,else is not
bool ReferenceLineInfo::IsChangeLanePath() const {
  return !Lanes().IsOnSegment();
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }
const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return sl_boundary_info_.adc_sl_boundary_;
}

const SLBoundary& ReferenceLineInfo::VehicleSlBoundary() const {
  return sl_boundary_info_.vehicle_sl_boundary_;
}

bool ReferenceLineInfo::
AddObstacleHelper(const Obstacle* obstacle) {
  return AddObstacle(obstacle) != nullptr;
}

PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle)
{
  if (!obstacle) {
    std::cout << "The provided obstacle is empty";
    return nullptr;
  }

  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));

  if(!path_obstacle){
    std::cout << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                     &perception_sl)) {
    std::cout<<"Failed to get sl boundary for obstacle: "<< obstacle->Id()
             <<endl;
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);

#if 0
  if (IsUnrelaventObstacle(path_obstacle))
  {
    ObjectDecisionType ignore;
    ignore.has_ignore = true;
    //ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    std::cout <<"NO build reference line st boundary. id:" << obstacle->Id()
              <<endl;
  } else {
    std::cout <<"build reference line st boundary. id:" << obstacle->Id()
              <<endl;
    path_obstacle->BuildReferenceLineStBoundary(
                 reference_line_, sl_boundary_info_.adc_sl_boundary_.start_s);

    std::cout << "reference line st boundary: "
           << path_obstacle->reference_line_st_boundary().min_t() << ", "
           << path_obstacle->reference_line_st_boundary().max_t()
           << ", s_max: " << path_obstacle->reference_line_st_boundary().max_s()
           << ", s_min: "
           << path_obstacle->reference_line_st_boundary().min_s()<<endl;
  }
#endif
  return path_obstacle;
}

bool ReferenceLineInfo::AddObstacles(const std::vector<const Obstacle*>& obstacles)
{
  if (g_config_param.use_multi_thread_to_add_obstacles) {
      cout<<"Error use_multi_thread_to_add_obstacles fuciotn don't open "<<endl;
      return false;
//      std::vector<std::future<bool>> futures;
//      for (const auto* obstacle : obstacles)
//      {
//        futures.push_back(ThreadPool::pool()->push(
//            std::bind(&ReferenceLineInfo::AddObstacleHelper, this, obstacle)));
//      }

//      for (const auto& f : futures) {
//        f.wait();
//      }

//      for (auto& f : futures) {
//        if (!f.get()) {
//          return false;
//        }
//      }
    } else {
      for (const auto* obstacle : obstacles) {
        if (!AddObstacle(obstacle))
        {
          cout << "Failed to add obstacle " << obstacle->Id()<<endl;
          return false;
        }
      }
    }

    return true;
}

bool ReferenceLineInfo::IsUnrelaventObstacle(PathObstacle* path_obstacle) {
  // if adc is on the road, and obstacle behind adc, ignore
  if (path_obstacle->PerceptionSLBoundary().start_s >reference_line_.Length())
  { //should be path_obstacle->PerceptionSLBoundary().start_s ???
    return true;
  }

  if (is_on_reference_line_ &&
      path_obstacle->PerceptionSLBoundary().end_s <
      sl_boundary_info_.adc_sl_boundary_.start_s//should be adc_sl_boundary_.start_s???
      &&reference_line_.IsOnLane(path_obstacle->PerceptionSLBoundary()) ) {
    return true;
  }

  return false;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

double ReferenceLineInfo::TrajectoryLength() const {
  const auto& tps = discretized_trajectory_.trajectory_points();
  if (tps.empty()) {
    return 0.0;
  }
  return tps.back().path_point.s;
}

void ReferenceLineInfo::SetStopPoint(const StopPoint& stop_point) {
  planning_target_.stop_point = stop_point;
  planning_target_.has_stop_point = true;
}

void ReferenceLineInfo::SetCruiseSpeed(double speed) {
  planning_target_.cruise_speed = speed;
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const
{
  if (reference_line_.reference_points().empty()) {
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =previous_reference_line_info.reference_line();

  SLPoint sl_point;
  Vec2d start_point_vec2d(start_point.x,start_point.y);

  prev_reference_line.XYToSL(start_point_vec2d, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
}

//const PathData& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedData& ReferenceLineInfo::speed_data() const { return speed_data_; }

//PathData* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedData* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }


//bool ReferenceLineInfo::IsRightTurnPath() const{

//   double route_s = 0.0;
//   const double adc_s = sl_boundary_info_.adc_sl_boundary_.end_s;
//   constexpr double kRightTurnStartBuff = 1.0;
//   for (const auto& seg : Lanes()) {
//     if (route_s > adc_s + kRightTurnStartBuff) {
//       break;
//     }
//     route_s += seg.end_s - seg.start_s;
//     if (route_s < adc_s) {
//       continue;
//     }
//     const auto& turn = seg.lane->lane().turn;
//     if (turn == map_lane::RIGHT_TURN) {
//       return true;
//     }
//   }
//   return false;
//}

void ReferenceLineInfo::ExportDecision(DecisionResult* decision_result) const {

//    MakeDecision(decision_result);
//    ExportTurnSignal(decision_result->mutable_vehicle_signal());
//    auto* main_decision = decision_result->mutable_main_decision();
//    if (main_decision->has_stop()) {
//      main_decision->mutable_stop()->set_change_lane_type(
//          Lanes().PreviousAction());
//    } else if (main_decision->has_cruise()) {
//      main_decision->mutable_cruise()->set_change_lane_type(
//          Lanes().PreviousAction());
//    }
}

void ReferenceLineInfo::MakeDecision(DecisionResult* decision_result) const {
  //CHECK_NOTNULL(decision_result);
  decision_result->main_decision.target_lane.clear();
  decision_result->object_decision.decision.clear();
  // cruise by default
  //decision_result->mutable_main_decision()->mutable_cruise();
  // check stop decision
  int error_code = MakeMainStopDecision(decision_result);
  if (error_code < 0) {
    MakeEStopDecision(decision_result);
  }
  MakeMainMissionCompleteDecision(decision_result);
  SetObjectDecisions(&decision_result->object_decision);
}

int ReferenceLineInfo::MakeMainStopDecision(DecisionResult* decision_result) const
{
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto path_obstacle : path_decision_.path_obstacles().Items())
  {
    const auto& obstacle = path_obstacle->obstacle();
    const auto& object_decision = path_obstacle->LongitudinalDecision();
    if (!object_decision.has_stop) {
        continue;
    }

    PointENU stop_point = object_decision.stop.stop_point;
    SLPoint stop_line_sl;
    reference_line_.XYToSL({stop_point.x, stop_point.y}, &stop_line_sl);

    double stop_line_s = stop_line_sl.s;
    if (stop_line_s < 0 || stop_line_s > reference_line_.Length()) {
      cout   << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line_.Length()
             << "]"<<endl;
      continue;
    }

    // check stop_line_s vs adc_s
    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop);
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =&decision_result->main_decision.task.stop;

    main_stop->reason_code = stop_decision->reason_code;
    main_stop->reason = ("stop by " + std::to_string(stop_obstacle->Id()));
    main_stop->stop_point.x = stop_decision->stop_point.x;
    main_stop->stop_point.y = stop_decision->stop_point.y;
    main_stop->stop_heading = stop_decision->stop_heading;

    cout   << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point.x<< stop_decision->stop_point.y
           << " ) stop_heading: " << stop_decision->stop_heading<<endl;

    return 1;
  }

  return 0;
}

void ReferenceLineInfo::MakeEStopDecision(DecisionResult* decision_result) const
{
  //decision_result->Clear();
  decision_result->main_decision.target_lane.clear();
  decision_result->object_decision.decision.clear();

  MainEmergencyStop* main_estop =&decision_result->main_decision.task.estop;

  main_estop->reason_code = ReasonCode::ESTOP_REASON_INTERNAL_ERR;
  main_estop->reason = ("estop reason to be added");
  //main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions = &decision_result->object_decision;

  for (const auto path_obstacle : path_decision_.path_obstacles().Items())
  {
    ObjectDecision object_decision;
    const auto& obstacle = path_obstacle->obstacle();

    object_decision.id = obstacle->Id();
    object_decision.perception_id = obstacle->PerceptionId();
    //object_decision->object_decision()->mutable_avoid();

    object_decisions->decision.push_back(object_decision);
  }
}

void ReferenceLineInfo::MakeMainMissionCompleteDecision(
                                     DecisionResult* decision_result) const
{
  if (!decision_result) {
    return;
  }
  auto main_stop = decision_result->main_decision.task.stop;
  if (main_stop.reason_code != STOP_REASON_DESTINATION) {
    return;
  }
  const auto& adc_pos = adc_planning_point_.path_point;
  if (math::util::DistanceXY(adc_pos, main_stop.stop_point) >
      g_config_param.destination_check_distance ) {
    return;
  }

  auto mission_complete =
                    &decision_result->main_decision.task.mission_complete;
  if (ReachedDestination()) {
    AdapterManager::GetPlanningStatus()->destination.has_passed_destination = true;
  } else {
    mission_complete->stop_point = main_stop.stop_point;
    mission_complete->stop_heading = main_stop.stop_heading;
  }
}

bool ReferenceLineInfo::ReachedDestination() const {
  constexpr double kDestinationDeltaS = 0.05;
  const auto* dest_ptr = path_decision_.Find(g_config_param.destination_obstacle_id);
  if (!dest_ptr) {
    return false;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop) {
    return false;
  }
  if (!reference_line_.IsOnLane(
          dest_ptr->obstacle()->PerceptionBoundingBox().center())) {
    return false;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s +
                        dest_ptr->LongitudinalDecision().stop.distance_s;

  return sl_boundary_info_.adc_sl_boundary_.end_s+kDestinationDeltaS >stop_s;
}

void ReferenceLineInfo::SetObjectDecisions(ObjectDecisions* object_decisions) const
{
  for (const auto path_obstacle : path_decision_.path_obstacles().Items()) {
    if (!path_obstacle->HasNonIgnoreDecision()) {
      continue;
    }

    ObjectDecision object_decision ;

    const auto& obstacle = path_obstacle->obstacle();
    object_decision.id = obstacle->Id();
    object_decision.perception_id = obstacle->PerceptionId();
    if (path_obstacle->HasLateralDecision() && !path_obstacle->IsLateralIgnore())
      {
        object_decision.object_decision.push_back(path_obstacle->LateralDecision());
      }
    if (path_obstacle->HasLongitudinalDecision() &&!path_obstacle->IsLongitudinalIgnore())
      {
        object_decision.object_decision.push_back(path_obstacle->LongitudinalDecision());
      }
    object_decisions->decision.push_back(object_decision);
  }
}

void ReferenceLineInfo::ExportEngageAdvice(EngageAdvice* engage_advice) const {
  constexpr double kMaxAngleDiff = M_PI / 6.0;
  auto* prev_advice = &AdapterManager::GetPlanningStatus()->engage_advice;
  if (!(&prev_advice->advice)) {
    prev_advice->advice = DISALLOW_ENGAGE;
  }

  if (!IsDrivable()) {
    if (prev_advice->advice == DISALLOW_ENGAGE) {
      prev_advice->advice = DISALLOW_ENGAGE;
    } else {
      prev_advice->advice = PREPARE_DISENGAGE;
    }
      prev_advice->reason = "Reference line not drivable";
  } else if (!is_on_reference_line_) {
    if (prev_advice->advice == DISALLOW_ENGAGE) {
      prev_advice->advice = DISALLOW_ENGAGE;
    } else {
      prev_advice->advice = PREPARE_DISENGAGE;
    }
    prev_advice->reason = "Not on reference line";
  } else {
    // check heading
    auto ref_point = reference_line_.GetReferencePoint(
                                     sl_boundary_info_.adc_sl_boundary_.end_s);

    if (AngleDiff(vehicle_state_.heading, ref_point.theta) >kMaxAngleDiff)
    {
      if (prev_advice->advice == DISALLOW_ENGAGE) {
        prev_advice->advice = DISALLOW_ENGAGE;
      } else {
        prev_advice->advice = PREPARE_DISENGAGE;
      }
      prev_advice->reason = "Vehicle heading is not aligned";
    } else {
      if (vehicle_state_.driving_mode != COMPLETE_AUTO_DRIVE) {
        prev_advice->advice = READY_TO_ENGAGE;
      } else {
        prev_advice->advice = KEEP_ENGAGED;
      }
      prev_advice->reason = "";
    }
  }
  engage_advice = prev_advice;
}

bool ReferenceLineInfo::
GetStopDistanceWithObstacle(double &stop_dis,std::vector<const Obstacle *> obstacles) const {

    double max_planning_dis = g_config_param.planning_upper_speed_limit *
           g_config_param.trajectory_time_length;
    double max_dis = max_planning_dis +  vehicle_sl_.s + g_vehicle_config.length/2.0;
    stop_dis = max_dis + 1;
    double max_sample_width = 1.2;
    double half_sample_width = max_sample_width/2.0;
    double saft_with = g_vehicle_config.width/2 + 0.1;
    double max_l_obs_r,min_l_obs_r,
           max_l_obs_l,min_l_obs_l;
    double center_dis = 0;
    if(!IsSideSlipClean()){
        max_l_obs_r = half_sample_width + saft_with;//sameple at left, obs on right
        min_l_obs_r = half_sample_width - saft_with;
        max_l_obs_l = -half_sample_width + saft_with;//sameple at right, obs on left
        min_l_obs_l = -half_sample_width - saft_with;

    }else{
       center_dis = half_sample_width;
       max_l_obs_r = max_sample_width + saft_with;//sameple at left, obs on right
       min_l_obs_r = max_sample_width - saft_with;
       max_l_obs_l = 0 + saft_with;//sameple at right, obs on left
       min_l_obs_l = 0 - saft_with;
    }

    for(const auto obs : obstacles){
       if( obs->IsStatic()){
          auto corners = obs->PerceptionPolygon().points();
          SLPoint obs_center_sl ,sl_point ;
          if(!route_.GetProjection(obs->PerceptionBoundingBox().center(),
                                   obs_center_sl.s,obs_center_sl.l))
              continue;
          for(const auto corner : corners) {
            if(!route_.GetProjection(corner,sl_point.s,sl_point.l))
                continue;
            if(sl_point.s < vehicle_sl_.s || sl_point.s > max_dis)
                continue;

            if(obs_center_sl.l < center_dis){ //obs on right
              if(sl_point.l > max_l_obs_r || sl_point.l < min_l_obs_r) continue;
            } else { //obs on left
              if(sl_point.l > max_l_obs_l || sl_point.l < min_l_obs_l) continue;
            }
            stop_dis = min(stop_dis,sl_point.s);
          }

       }else{
           continue;
       }
    }

    return stop_dis < max_dis;

}


SLBoundary ReferenceLineInfo::ComputeObstacleBoundary(
  const std::vector<Vec2d>& vertices,
  const std::vector<PathPoint>& discretized_ref_points) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s  (std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l  (std::numeric_limits<double>::lowest());

  for (const auto& point : vertices) {
    auto sl_point = math::PathMatcher::GetPathFrenetCoordinate(
        discretized_ref_points, point.x(), point.y());
    start_s = std::fmin(start_s, sl_point.first);
    end_s = std::fmax(end_s, sl_point.first);
    start_l = std::fmin(start_l, sl_point.second);
    end_l = std::fmax(end_l, sl_point.second);
  }

  SLBoundary sl_boundary;
  sl_boundary.start_s=(start_s);
  sl_boundary.end_s=(end_s);
  sl_boundary.start_l=(start_l);
  sl_boundary.end_l=(end_l);

  return sl_boundary;
}


SampleRecommendation ReferenceLineInfo::
SamepleDecider(std::vector<const Obstacle *> obstacles)const{
    
   std::vector<const Obstacle *> consider_obstacles;
   double valid_obs_range_start_s = vehicle_sl_.s - 1.0;
   double valid_obs_range_end_s = vehicle_sl_.s + g_config_param.decision_horizon;
   for(auto obstacle : obstacles) {
      //get obstacle geometry polygon
      const Polygon2d& polygon = obstacle->PerceptionPolygon();
      int32_t obstacle_id = obstacle->Id();
      //Compute Obstacle SL Boundary
      SLBoundary sl_boundary = ComputeObstacleBoundary(
        polygon.GetAllVertices(), reference_line().reference_points());

      double left_width  = g_config_param.default_reference_line_width * 0.5;
      double right_width = g_config_param.default_reference_line_width * 0.5;
      reference_line().GetLaneWidth(sl_boundary.start_s,left_width, right_width);
      //obstacle out of way range continue next obstacle
      if (sl_boundary.start_s > valid_obs_range_end_s ||
          sl_boundary.end_s < valid_obs_range_start_s ||
          sl_boundary.start_l >left_width ||
          sl_boundary.end_l < -right_width) {
        cout << "Obstacle [" << obstacle_id << "] is out of range."<<endl;
        continue;
      }
     consider_obstacles.push_back(obstacle);
   }


    SampleRecommendation result_out;
    double max_planning_dis = g_config_param.planning_upper_speed_limit *
           g_config_param.trajectory_time_length + vehicle_sl_.s+10;
    double max_obs_length = 15.0;
    double max_dis = max_planning_dis  + max_obs_length;
    result_out.sl_point.s = max_dis + 1;

    double sample_width = g_config_param.sample_width;
    double saft_with = g_vehicle_config.width/2 + g_config_param.lat_collision_buffer;
    double max_l_obs_r,min_l_obs_r,
           max_l_obs_l,min_l_obs_l;
    double center_dis = 0;
    if(IsSideSlipClean()){
        center_dis  = sample_width;
        max_l_obs_r = sample_width + saft_with;//sameple at left, obs on right
        min_l_obs_r = sample_width - saft_with;
        max_l_obs_l = 0 + saft_with;//sameple at right, obs on left
        min_l_obs_l = 0 - saft_with;

    }else{
       max_l_obs_r =  sample_width + saft_with;//sameple at left, obs on right
       min_l_obs_r =  sample_width - saft_with;
       max_l_obs_l = -sample_width + saft_with;//sameple at right, obs on left
       min_l_obs_l = -sample_width - saft_with;

    }

    for(const auto obs : consider_obstacles){
       double min_S = 1000;
       SampleRecommendation result;
       result.sl_point.s = max_dis + 2;
       if( 1 /*obs->IsStatic()*/){
          auto corners = obs->PerceptionPolygon().points();
          SLPoint obs_center_sl ,sl_point ;
          if(!route_.GetProjection(obs->PerceptionBoundingBox().center(),
                                   obs_center_sl.s,obs_center_sl.l))
              continue;
          if(obs_center_sl.s > max_planning_dis)  continue;
          for(const auto corner : corners) {
            if(!route_.GetProjection(corner,sl_point.s,sl_point.l))
                continue;
            if(sl_point.s < vehicle_sl_.s - 2.0 || sl_point.s > max_dis)
                continue;
            min_S = min(min_S,sl_point.s);
            if(obs_center_sl.l < center_dis)
            { //obs on right
              if(sl_point.l < min_l_obs_r) {
                 if(sl_point.l < min_l_obs_r && sl_point.l > min_l_obs_l) {//right can't pass
                    if (result.pass_type != result.STOP_DRIVR ){//dosen't have stop point
                       if(sl_point.s < result.sl_point.s){
                           result.pass_type = result.LEFT_PASS;
                           result.sl_point = sl_point; }
                    } else if(sl_point.s + 4.0 < result.sl_point.s ){
                       //if has stop point and far away side pass point,
                       //update side pass point
                       result.pass_type = result.LEFT_PASS;
                       result.sl_point = sl_point;
                    } else{
                        //if has stop point and near side pass point update
                        //stop point for a nearest dis
                        if(sl_point.s < result.sl_point.s){
                           result.pass_type = result.STOP_DRIVR;
                           result.sl_point = sl_point; }
                    }
                    continue;
                 } else{ continue; }
              }
            } else { //obs on left
              if(sl_point.l > max_l_obs_l) {
                if(sl_point.l > max_l_obs_l && sl_point.l < max_l_obs_r) {//left can't pass
                   if (result.pass_type != result.STOP_DRIVR ){//dosen't have stop point
                      if(sl_point.s < result.sl_point.s){
                          result.pass_type = result.RIGHT_PASS;
                          result.sl_point = sl_point; }
                   } else if(sl_point.s + 4.0 < result.sl_point.s ){
                       //if has stop point and far away side pass point,
                       //update side pass point
                      result.pass_type = result.RIGHT_PASS;
                      result.sl_point = sl_point;
                   } else{
                      //if has stop point and near side pass point update
                      //stop point for a nearest dis
                      if(sl_point.s < result.sl_point.s){
                         result.pass_type = result.STOP_DRIVR;
                         result.sl_point = sl_point; }
                   }
                   continue;
                } else{ continue; }
              }
           }


           result.pass_type = result.STOP_DRIVR;
           result.sl_point = sl_point;
       }

    }

    if(result.pass_type != result.NONE) {
      result.sl_point.s = min_S;
    }else{
        continue;
    }

    if(result_out.pass_type == result_out.NONE){

        result_out = result;
        continue;
    }

    //case 1 two result all is stop select min dis
   if(result_out.pass_type == result_out.STOP_DRIVR &&
      result.pass_type != result.STOP_DRIVR &&
      result_out.sl_point.s > result.sl_point.s &&
      result_out.sl_point.s < result.sl_point.s + 6.0 ) {

      result_out.sl_point = result.sl_point;
    }else if(result.pass_type == result.STOP_DRIVR &&
             result_out.pass_type != result_out.STOP_DRIVR &&
             result.sl_point.s > result_out.sl_point.s &&
             result.sl_point.s < result_out.sl_point.s + 6.0) {
        result_out.pass_type = result_out.STOP_DRIVR;
    } else {
        result_out = result_out.sl_point.s < result.sl_point.s ? result_out : result;
    }

  }

  if(result_out.sl_point.s - vehicle_sl_.s > 10.5)
      result_out.pass_type = result_out.NONE;

  cout<<"pass type = "<<result_out.pass_type<<endl;
  cout<<"sl_point = ("<<result_out.sl_point.s<<", "
      <<result_out.sl_point.l<<")"<<endl;

  return result_out;
    
    
}

SampleRecommendation ReferenceLineInfo::
SamepleDeciderWithObsSL(std::vector<const Obstacle *> obstacles)const{

   std::vector<const Obstacle *> consider_obstacles;
   std::vector<pair<int,SLBoundary>> consider_obstacles_sl;
   double valid_obs_range_start_s = vehicle_sl_.s - 1.0;
   double valid_obs_range_end_s   = vehicle_sl_.s + g_config_param.decision_horizon;
   for(auto obstacle : obstacles) {
      //get obstacle geometry polygon
      const Polygon2d& polygon = obstacle->PerceptionPolygon();
      int32_t obstacle_id = obstacle->Id();
      //Compute Obstacle SL Boundary
      SLBoundary sl_boundary = ComputeObstacleBoundary(
        polygon.GetAllVertices(), reference_line().reference_points());

      double left_width  = g_config_param.default_reference_line_width * 0.5;
      double right_width = g_config_param.default_reference_line_width * 0.5;
      reference_line().GetLaneWidth(sl_boundary.start_s,left_width, right_width);
      left_width  -= g_config_param.obstacle_filter_buffer;
      right_width -= g_config_param.obstacle_filter_buffer;
      //obstacle out of way range continue next obstacle
      if (sl_boundary.start_s > valid_obs_range_end_s ||
          sl_boundary.end_s < valid_obs_range_start_s ||
          sl_boundary.start_l > left_width ||
          sl_boundary.end_l < -right_width ) {
        //cout << "Obstacle [" << obstacle_id << "] is out of range."<<endl;
        continue;
      }
     consider_obstacles.push_back(obstacle);
     consider_obstacles_sl.emplace_back(obstacle->Id(),sl_boundary);
   }

    double sample_width = g_config_param.sample_width;
    double saft_with = g_vehicle_config.width/2 +
                       g_config_param.lat_collision_buffer +
                       g_config_param.per_error_dis;
    double center_dis = 0;
    array<double,6> key_value;
    if(IsSideSlipClean()){
       center_dis = sample_width;
    }else{

       center_dis = 0;
    }
    key_value.at(0) = center_dis - sample_width - saft_with;
    key_value.at(1) = center_dis - saft_with;
    key_value.at(2) = center_dis - sample_width + saft_with;
    key_value.at(3) = center_dis + sample_width - saft_with;
    key_value.at(4) = center_dis + saft_with;
    key_value.at(5) = center_dis + sample_width + saft_with;


    vector<pair<pair<int,SLBoundary>,SampleRecommendation>> result_set;
    for(const auto obs_sl:consider_obstacles_sl){
        SampleRecommendation res;
       //1.no obs effecitve pass type
        if( obs_sl.second.end_l < key_value.at(0) ||
            obs_sl.second.start_l >   key_value.at(5)){
            continue;
        }
       //2.STOP
        if( obs_sl.second.end_l > key_value.at(3)&&
            obs_sl.second.start_l   < key_value.at(2)){
           res.pass_type = res.STOP_DRIVR;
           res.sl_point.s = obs_sl.second.start_s;
           res.sl_point.l = abs(obs_sl.second.start_l) < abs(obs_sl.second.end_l)?
                            obs_sl.second.start_l : obs_sl.second.end_l;
           result_set.emplace_back(obs_sl,res) ;
           continue;
        }
       //3.left and right pass
        if( (obs_sl.second.end_l > key_value.at(2)&&
             obs_sl.second.end_l < key_value.at(3)  ) &&
            (obs_sl.second.start_l   > key_value.at(2)&&
             obs_sl.second.start_l   < key_value.at(3)  )   ){
            res.pass_type =res.LEFT_RIGHT;
            res.sl_point.s = obs_sl.second.start_s;
            res.sl_point.l = abs(obs_sl.second.start_l) < abs(obs_sl.second.end_l)?
                             obs_sl.second.start_l : obs_sl.second.end_l;
           result_set.emplace_back(obs_sl,res) ;
           continue;
        }
        // left pass
         if( obs_sl.second.end_l < key_value.at(3) ){

            if(obs_sl.second.end_l > key_value.at(1)) {
        //4.only left pass
              res.pass_type =res.LEFT_PASS;
              res.sl_point.s = obs_sl.second.start_s;
              res.sl_point.l = obs_sl.second.end_l;
              result_set.emplace_back(obs_sl,res) ;
              continue;
            }else{
        //5.left and center pass
              res.pass_type =res.LEFT_CENTER_PASS;
              res.sl_point.s = obs_sl.second.start_s;
              res.sl_point.l = obs_sl.second.end_l;
              result_set.emplace_back(obs_sl,res) ;
              continue;
            }
         }
        // right pass
          if( obs_sl.second.start_l > key_value.at(2) ){

             if(obs_sl.second.start_l < key_value.at(4)) {
        //6.only right pass
               res.pass_type =res.RIGHT_PASS;
               res.sl_point.s = obs_sl.second.start_s;
               res.sl_point.l = obs_sl.second.start_l;
               result_set.emplace_back(obs_sl,res) ;
               continue;
             }else{
        //7.only right and center pass
               res.pass_type =res.RIGHT_CENTER_PASS;
               res.sl_point.s = obs_sl.second.start_s;
               res.sl_point.l = obs_sl.second.start_l;
               result_set.emplace_back(obs_sl,res) ;
               continue;
             }
          }

    }

   SampleRecommendation result_out,stop_res,result_sec;
   result_out.sl_point.s = 1000;
   stop_res.sl_point.s  = 1000;
   result_sec.sl_point.s = 1000;
   pair<int,SLBoundary> key_obs,key_sec_obs,key_stop_obs;
   for(const auto res :result_set){
       if( res.second.pass_type != res.second.NONE &&
           res.second.sl_point.s < result_out.sl_point.s ){
           result_out = res.second;
           key_obs = res.first;
           if(res.second.pass_type == res.second.STOP_DRIVR){
               stop_res = result_out;
               key_stop_obs = key_obs;
           }
       }
       if(res.second.pass_type  != res.second.NONE &&
          res.second.pass_type  != res.second.STOP_DRIVR &&
          res.second.sl_point.s != result_out.sl_point.s &&
          res.second.sl_point.s < result_sec.sl_point.s){
           result_sec = res.second;
           key_sec_obs = res.first;

       }
   }

  if( stop_res.sl_point.s > result_out.sl_point.s &&
      (stop_res.sl_point.s - result_out.sl_point.s <
       g_vehicle_config.front_edge_to_center +
       g_config_param.lat_collision_buffer /2.0 + 0.1) ){
      result_out.pass_type = result_out.STOP_DRIVR;
      cout<<"nearest obs maybe pass, but front obs can't pass !!! because obs id = "
          << key_stop_obs.first<<endl;
  }

  if(result_out.pass_type != result_out.STOP_DRIVR &&
     result_sec.pass_type != result_sec.STOP_DRIVR &&
     result_sec.pass_type != result_sec.NONE &&
     (result_sec.sl_point.s - result_out.sl_point.s <
      g_config_param.combine_stop_result_dis ||
      key_sec_obs.second.start_s - key_obs.second.end_s <
      g_config_param.combine_stop_result_dis )     )
  {
     if((result_out.sl_point.l > key_value.at(3)&&
         result_sec.sl_point.l < key_value.at(2)) ||
        (result_sec.sl_point.l > key_value.at(3)&&
         result_out.sl_point.l < key_value.at(2))){
         cout<<"sec obs id = "<<key_sec_obs.first<<endl;
         cout<<"sec obs sl = ("<<result_sec.sl_point.s<<","
             <<result_sec.sl_point.l<<" )"<<endl;
         cout<<"key obs id = "<<key_obs.first<<endl;
         cout<<"key obs sl = ("<<result_out.sl_point.s<<","
             <<result_out.sl_point.l<<" )"<<endl;
         cout<<"two obstacles make stop reslut"<<endl;
         result_out.pass_type = result_out.STOP_DRIVR;


     }

  }

  if(result_out.sl_point.s - vehicle_sl_.s >
     g_config_param.decider_sample_dis){
      result_out.pass_type = result_out.NONE;
  }else if(result_out.pass_type != result_out.STOP_DRIVR &&
           (result_out.sl_point.s - vehicle_sl_.s >
            g_config_param.valid_passing_obs_dis)){
      result_out.pass_type = result_out.NONE;
  }else{
      cout<<"pass type = "<<result_out.pass_type<<endl;
      cout<<"key S = "<<result_out.sl_point.s<<endl;
      cout<<"(min_l, max_l) = ("<<key_obs.second.start_l<<", "
                                <<key_obs.second.end_l<<")"<<endl;
      cout<<"key obs id = "<<key_obs.first<<endl;

  }

  return result_out;


}

SLPoint ReferenceLineInfo::GetSLPoint(Vec2d point) const{

    auto sl = math::PathMatcher::GetPathFrenetCoordinate(
            reference_line().reference_points(), point.x(), point.y());

    SLPoint sl_point;
    sl_point.s = sl.first;
    sl_point.l = sl.second;

    return sl_point;
}


}//end namespace planning
