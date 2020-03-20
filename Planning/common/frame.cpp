
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <list>
#include <string>
#include <utility>

#include "./frame.h"
#include "../math/vec2d.h"
#include "./vehicle_state/vehicle_state_provider.h"
#include "../reference_line/reference_line_provider.h"
#include "garbage_deal.h"

extern ConfigParam   g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {

constexpr double kMathEpsilon = 1e-8;

Frame::Frame(uint32_t sequence_num,
             const TrajectoryPoint &planning_start_point,
             const double start_time,
             const VehicleState &vehicle_state,
             ReferenceLineProvider *reference_line_provider,
             const vector<Garbage> &Garbages)
    : sequence_num_(sequence_num),
      planning_start_point_(planning_start_point),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      reference_line_provider_(reference_line_provider),
      garbages_(Garbages)
{
  if (g_config_param.enable_lag_prediction) {//true
    lag_predictor_.reset(
            new LagPrediction(g_config_param.lag_prediction_min_appear_num,//5
                              g_config_param.lag_prediction_max_disappear_num));//3
  }
}

common::Status Frame::Init() {

 //2. get vehicle_state_
  Point3D point = {vehicle_state_.x, vehicle_state_.y, vehicle_state_.z};
  if (std::isnan(point.x) || std::isnan(point.y)) {
    cout << "init point is not set" << endl;
    return common::PLANNING_ERROR;
  }
  cout << "Enabled align prediction time ? : "
       << std::boolalpha<< g_config_param.align_prediction_time<<endl;
 //3. prediction

  if ( AdapterManager::GetPrediction()&&
       (!AdapterManager::GetPredictionLists()->empty()) )
  {
    if (g_config_param.enable_lag_prediction && lag_predictor_ ) {
        lag_predictor_->GetLaggedPrediction(&prediction_);
    } else {
      prediction_ = * AdapterManager::GetLatestObserved();
    }

    if (g_config_param.align_prediction_time) {
      AlignPredictionTime(vehicle_state_.timestamp_ms/1000.0,&prediction_);
    }

    for (auto &ptr : Obstacle::CreateObstacles(prediction_)) {
      AddObstacle(*ptr);
    }
  }

  if (g_config_param.enable_collision_detection) {//false
    const auto *collision_obstacle = FindCollisionObstacle();
    if (collision_obstacle) {
      cout<<"Found collision with obstacle: "<<collision_obstacle->Id()<<endl;
      return common::PLANNING_ERROR;
    }
  }
 //4. Create Reference Line
  if (!CreateReferenceLineInfo()) {
    cout << "Failed to init reference line info"<<endl;
    return common::PLANNING_ERROR;
  }


  return common::OK;
}

bool Frame::CreateReferenceLineInfo()
{
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegment> segments;
  std::vector<hdmap::ReferenLineWithParam> ref_line_with_params;
  reference_lines.clear();
  segments.clear();
  if (!reference_line_provider_->GetReferenceLines(&reference_lines, &segments,
                                                   &ref_line_with_params))
  {
    cout<< "Failed to create reference line"<<endl;
    return false;
  }

  if(reference_lines.size() != segments.size())
    {
      cout<<"Error reference_lines.size() != segments.size()"<<endl;
//      return false;
    }

  reference_line_info_.clear();
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end())
  {
    if (segments_iter->StopForDestination())
      {
         is_near_destination_ = true;
      }


    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                             *ref_line_iter, *segments_iter);
    ///construct garbages class
    GarbageDeal garbage_deal(&reference_line_info_.back(),garbages_);
    ///set garbages sl_point
    reference_line_info_.back().SetGarbagesSL(garbage_deal.GetGarbagesSL());

    ++ref_line_iter;
    ++segments_iter;
  }

  if (reference_line_info_.size() == 2)
  {
    Vec2d xy_point(vehicle_state_.x, vehicle_state_.y);
    SLPoint first_sl;
    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,&first_sl))
     {  return false; }
    SLPoint second_sl;
    if (!reference_line_info_.back().reference_line().XYToSL(xy_point, &second_sl))
     { return false;  }
    const double offset = first_sl.l - second_sl.l;
    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
  }

  bool has_valid_reference_line = false;
  for (auto &ref_info : reference_line_info_) {
    if (!ref_info.Init(obstacles())) {
      cout << "Failed to init reference line"<<endl;
      continue;
    } else {
      has_valid_reference_line = true;
    }
  }

 if(segments.size() == 1){
      if(segments.front().GetSideType()){
         route_type_ = SINGLE_SLIDE;
      }else{
         route_type_ = SINGLE_ROUTE;
      }
  }else if(segments.size() == 2){
      if(segments.front().GetSideType() ||segments.back().GetSideType()){
         route_type_ = DOUBLE_SINGLE_SLIDE;
      }else{
         route_type_ = DOUBLE_ROUTE;
      }
 }else{
         route_type_ = ROUTE_UNKNOWN;
         cout<<"Error route type is unknown, edge constraint maybe is invalid"<<endl;
 }

  no_using_right_back_ultra_ = false;
  for (const auto &ref_info : reference_line_info_) {
      if(ref_info.Lanes().IsOnSegment() && ref_info.Lanes().GetSideType()){
         auto ego_sl = ref_info.GetVechicleSLPoint();
         if(ego_sl.l < g_config_param.sample_width/2.0){
             no_using_right_back_ultra_ = true;
             break;
         }
      }
  }

  return has_valid_reference_line;
}

const Obstacle *Frame::CreateStopObstacle(
                                           ReferenceLineInfo *const reference_line_info,
                                           const int32_t &obstacle_id,
                                           const double obstacle_s) {
  if (reference_line_info == nullptr) {
    cout << "reference_line_info nullptr"<<endl;
    return nullptr; }

  const auto &reference_line = reference_line_info->reference_line();
  const double box_center_s=obstacle_s + g_config_param.virtual_stop_wall_length/2.0;//0.1
  auto  path_point = reference_line.GetReferencePoint(box_center_s);
  Vec2d box_center(path_point.x,path_point.y);
  double heading = reference_line.GetReferencePoint(obstacle_s).theta;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  reference_line.GetLaneWidth(obstacle_s, lane_left_width, lane_right_width);
  Box2d stop_wall_box{box_center, heading, g_config_param.virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

const Obstacle* Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const int32_t &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {

  if (reference_line_info == nullptr) {
    cout << "reference_line_info nullptr"<<endl;
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  SLPoint sl_point;
  sl_point.s = obstacle_start_s;
  sl_point.l = 0.0;
  Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    cout << "Failed to get start_xy from s,l: " << sl_point.s <<","<<sl_point.l<<endl;
    return nullptr;
  }

  // end_xy
  sl_point.s= obstacle_end_s;
  sl_point.l= 0.0;
  Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    cout << "Failed to get end_xy from s,l: " << sl_point.s <<","<<sl_point.l<<endl;
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, left_lane_width,
                                   right_lane_width)) {
    cout<< "Failed to get lane width at s[" << obstacle_start_s << "]"<<endl;
    return nullptr;
  }
  Box2d obstacle_box{ LineSegment2d(obstacle_start_xy, obstacle_end_xy),
                      left_lane_width + right_lane_width };
  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle* Frame::CreateStaticVirtualObstacle(const int32_t &id,
                                                   const Box2d &box) {
  const auto *object = obstacles_.Find(id);
  if (object) {
    cout << "obstacle " << id << " already exist."<<endl;
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    cout << "Failed to create virtual obstacle " << id<<endl;
  }
  return ptr;
}

const Obstacle *Frame::FindCollisionObstacle() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  const auto &param =g_vehicle_config;//获取车辆参数

  Vec2d position(vehicle_state_.x, vehicle_state_.y);
  Vec2d vec_to_center(
      (param.front_edge_to_center - param.back_edge_to_center) / 2.0,
      (param.left_edge_to_center - param.right_edge_to_center) / 2.0);

  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading));
  Box2d adc_box(center, vehicle_state_.heading, param.length,param.width);//画出车的边框

  const double adc_half_diagnal = adc_box.diagonal() / 2.0;//中心到 边框角点的距离
  for (const auto &obstacle : obstacles_.Items()) {

    if (obstacle->IsVirtual()) { continue; }

    double center_dist =//车中心到障碍物中心距离
        adc_box.center().DistanceTo(obstacle->PerceptionBoundingBox().center());

    if (center_dist > obstacle->PerceptionBoundingBox().diagonal() / 2.0 +
        adc_half_diagnal + g_config_param.max_collision_distance)
     { //max_collision_distance = 0.1
       cout << "Obstacle : " << obstacle->Id() << " is too far to collide"<<endl;
       continue; }

    double distance = obstacle->PerceptionPolygon().DistanceTo(adc_box);
    if (g_config_param.ignore_overlapped_obstacle && distance < kMathEpsilon)//1*e^(-10)
     {
       bool all_points_in = true;
       for (const auto &point : obstacle->PerceptionPolygon().points())
        {
         if (!adc_box.IsPointIn(point)) {
             all_points_in = false;
             break; }
        }
       if (all_points_in) {
         cout << "Skip overlapped obstacle, which is often caused by lidar "
                 "calibration error"<<endl;
         continue;
       }
    }

    if (distance < g_config_param.max_collision_distance) { //0.1
      cout << "Found collision with obstacle " << obstacle->Id()<<endl;
      return obstacle; }

  }
  return nullptr;
}

void Frame::AlignPredictionTime(const double planning_start_time,
                            prediction::PredictionObstacles *prediction_obstacles) {
  if (prediction_obstacles == NULL)
  {
    cout<<"ERROR prediction_obstacles == NULL"<<endl;
    return;
  }
  double prediction_header_time = prediction_obstacles->start_timestamp /1000.0;

  for (auto &obstacle : (*prediction_obstacles).prediction_obstacle)
   {
    for (auto &trajectory : obstacle.trajectory)
    {
     for (auto &point : trajectory.trajectory_point) {
      point.relative_time=(prediction_header_time + point.relative_time -
                                                            planning_start_time);
      }
     if (!trajectory.trajectory_point.empty() &&
         trajectory.trajectory_point.begin()->relative_time < 0)
       {
         auto it = trajectory.trajectory_point.begin();//删除小于零的一段
         while (it != trajectory.trajectory_point.end() &&it->relative_time < 0)
         {  ++it;  }
         trajectory.trajectory_point.erase(trajectory.trajectory_point.begin(), it);
       }
     }
  }
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo()
{
  double min_cost = std::numeric_limits<double>::infinity();
  drive_reference_line_info_ = nullptr;

  for (const auto &reference_line_info : reference_line_info_) {
    if (reference_line_info.IsDrivable() &&
        reference_line_info.Cost() < min_cost) {
      drive_reference_line_info_ = &reference_line_info;
      min_cost = reference_line_info.Cost();
    }
  }
  cout<<"min_cost = "<<min_cost<<endl;
  return drive_reference_line_info_;
}

void Frame::AddObstacle(const Obstacle &obstacle)
{
  obstacles_.Add(obstacle.Id(), obstacle);
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

Obstacle *Frame::Find(const int32_t &id) { return obstacles_.Find(id); }

const TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

const VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

std::list<ReferenceLineInfo> &Frame::reference_line_info() {
  return reference_line_info_;
}


const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}


//don't use Frame::UpdateReferenceLinePriority
//find lanes id corespond pair id and seting reference_line_info priority value
//void Frame::UpdateReferenceLinePriority(const std::map<int32, uint32_t> &id_to_priority)
//{
//  for (const auto &pair : id_to_priority) {
//    const auto id = pair.first;
//    const auto priority = pair.second;
//    auto ref_line_info_itr =
//        std::find_if(reference_line_info_.begin(), reference_line_info_.end(),
//                     [&id](const ReferenceLineInfo &ref_line_info) {
//                       return ref_line_info.Lanes().Id() == id;//
//                     });
//    if (ref_line_info_itr != reference_line_info_.end()) {
//      ref_line_info_itr->SetPriority(priority);
//    }
//  }
//}


} //namespace planning
