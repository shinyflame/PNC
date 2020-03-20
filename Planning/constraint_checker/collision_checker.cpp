
#include <array>
#include <cmath>
#include <utility>

#include "../constraint_checker/collision_checker.h"
#include "../math/path_matcher.h"
#include "../math/vec2d.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {


CollisionChecker::CollisionChecker(
    const std::vector<const Obstacle*>& obstacles,
    const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line,
    const ReferenceLineInfo* ptr_reference_line_info,
    const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph,
    RouteType route_type) {
  ptr_reference_line_info_ = ptr_reference_line_info;
  ptr_path_time_graph_ = ptr_path_time_graph;
  ComputeSafeEdge(route_type);
  BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d,
                            discretized_reference_line);
}
void CollisionChecker::ComputeSafeEdge(RouteType route_type){

    slide_clean_right_min_ = -g_config_param.default_slide_center_to_right_dis;
    slide_clean_left_max_  = 2*g_config_param.default_reference_line_width;
    non_slide_right_min_ = - 2*g_config_param.default_reference_line_width;
    non_slide_left_max_ =    2*g_config_param.default_reference_line_width;


    if(route_type == DOUBLE_SINGLE_SLIDE){
      slide_clean_right_min_ = -( g_config_param.default_slide_center_to_right_dis +
                                 g_config_param.max_beyond_side_buffer );
      slide_clean_left_max_ = 2 * g_config_param.default_reference_line_width -
                                 g_config_param.default_slide_center_to_right_dis +
                                 g_config_param.max_beyond_side_buffer;

      non_slide_right_min_ = -(g_config_param.default_reference_line_width*1.5 +
                              g_config_param.max_beyond_side_buffer);
      non_slide_left_max_ =    g_config_param.default_reference_line_width / 2.0 +
                              g_config_param.max_beyond_side_buffer;
    }else if(route_type == SINGLE_SLIDE){
      slide_clean_right_min_ = -( g_config_param.default_slide_center_to_right_dis +
                                 g_config_param.max_beyond_side_buffer );
      slide_clean_left_max_ =     g_config_param.default_reference_line_width -
                                 g_config_param.default_slide_center_to_right_dis +
                                 g_config_param.max_beyond_side_buffer;
    }else if(route_type == SINGLE_ROUTE){

        non_slide_right_min_ = - 0.8*g_config_param.default_reference_line_width
                                   - g_config_param.max_beyond_side_buffer;
        non_slide_left_max_ =    0.8*g_config_param.default_reference_line_width
                                   + g_config_param.max_beyond_side_buffer;
    }

}
bool CollisionChecker::InCollision(
    const DiscretizedTrajectory& discretized_trajectory) {
  //CHECK_LE(discretized_trajectory.NumOfPoints(),
          // predicted_bounding_rectangles_.size());
  const auto& vehicle_config = g_vehicle_config;
  double ego_length = vehicle_config.length +
                      g_config_param.lon_collision_buffer;
  double ego_width  = vehicle_config.width +
                      2.0 * g_config_param.lat_collision_buffer;

  for (std::size_t i = discretized_trajectory.NumOfPoints()-1; i >=3 ; i-=3) {
    const auto& trajectory_point =
        discretized_trajectory.TrajectoryPointAt(i);
//    if(is_get_stop_obs_ && trajectory_point.path_point.s  > stop_s_)
//        return true;

    double ego_theta = trajectory_point.path_point.theta;
    Box2d ego_box(
        {trajectory_point.path_point.x, trajectory_point.path_point.y},
         ego_theta, ego_length, ego_width);
    Polygon2d ego_polygon(ego_box);
    Box2d car_box(
    {trajectory_point.path_point.x, trajectory_point.path_point.y},
     ego_theta, 2.5, 1.4);
    vector<Vec2d> ego_front_points;
    ego_front_points.push_back(car_box.GetAllCorners().at(0));
    ego_front_points.push_back(car_box.GetAllCorners().at(1));
    auto  ego_front_sl =
    ptr_reference_line_info_->ComputeObstacleBoundary(
    ego_front_points,
    ptr_reference_line_info_->reference_line().reference_points());
//    if(ptr_reference_line_info_->IsSideSlipClean()){
//        if( ego_front_sl.start_l < slide_clean_right_min_ ||
//            ego_front_sl.end_l   > slide_clean_left_max_ ){
//            cout<<"ego_front min_l and  max_l = ( "<<ego_front_sl.start_l
//                <<", "<< ego_front_sl.end_l<<" )"<<endl;
//            cout<<"Error ego_l on slide  beyond side"<<endl;

//            return true;
//        }
//    }else{
//        if( ego_front_sl.start_l < non_slide_right_min_ ||
//            ego_front_sl.end_l   > non_slide_left_max_){
//            cout<<"ego_front min_l and  max_l = ( "<<ego_front_sl.start_l
//                <<", "<< ego_front_sl.end_l<<" )"<<endl;
//            cout<<"Error ego_l non slide  beyond side"<<endl;
//            return true;
//        }
//    }
    //ego_box.ExtendInLength(-trans_dis,trans_dis);
    if(g_config_param.enable_ploygon_checking){
      // add polygons overlap estimate by sxl@20190902

      for(const auto& obstacle_polygon : predicted_polygons_[i]) {
         if(ego_polygon.HasOverlap(obstacle_polygon )) return true;
      }
    } else {
      for (const auto& obstacle_box : predicted_bounding_rectangles_[i]) {
        if (ego_box.HasOverlap(obstacle_box)) return true;
      }
    }
  }
  return false;
}

void CollisionChecker::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles,
    const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  //CHECK(predicted_bounding_rectangles_.empty());

  // If the ego vehicle is in lane,
  // then, ignore all obstacles from the same lane.
  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);
  std::vector<const Obstacle*> obstacles_considered;
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (ego_vehicle_in_lane &&
        (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s,
                                    discretized_reference_line) ||
         !ptr_path_time_graph_->IsObstacleInGraph(obstacle->Id()))) {
      continue;
    }

    obstacles_considered.push_back(obstacle);
  }

//  is_get_stop_obs_ =
//  ptr_reference_line_info_->GetStopDistanceWithObstacle(stop_s_,obstacles_considered);
//  if(is_get_stop_obs_){

//      stop_s_ -= ptr_reference_line_info_->GetVechicleSLPoint().s;
//  }

  double relative_time = 0.0;
  while (relative_time < g_config_param.trajectory_time_length)
  {
    std::vector<Box2d> predicted_env;
    std::vector<Polygon2d> predicted_polygon;
    for (const Obstacle* obstacle : obstacles_considered)
    {
      // If an obstacle has no trajectory, it is considered as static.
      // Obstacle::GetPointAtTime has handled this case.
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      //box.LongitudinalExtend(2.0 * g_config_param.lon_collision_buffer);//2.0
      //box.LateralExtend(2.0 * g_config_param.lat_collision_buffer);     // 0.1
      predicted_env.push_back(std::move(box));
      ///add static obstacle polygon for all obastacles change by sxl @20190902
      predicted_polygon.push_back(obstacle->PerceptionPolygon());
    }

    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    ///add static polygons for all obastacles change by sxl @20190902
    predicted_polygons_.push_back(std::move(predicted_polygon));

    relative_time += g_config_param.trajectory_time_resolution;//0.1s
  }
}

bool CollisionChecker::IsEgoVehicleInLane(
    const double ego_vehicle_s, const double ego_vehicle_d) {
  double left_width  = g_config_param.default_reference_line_width * 0.5;//4.0
  double right_width = g_config_param.default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      ego_vehicle_s, left_width, right_width);
  return ego_vehicle_d < left_width && ego_vehicle_d > -right_width;
}

bool CollisionChecker::IsObstacleBehindEgoVehicle(
    const Obstacle* obstacle,
    const double ego_vehicle_s,
    const std::vector<PathPoint>& discretized_reference_line)
{
  double half_lane_width = g_config_param.default_reference_line_width * 0.5;
  TrajectoryPoint point = obstacle->GetPointAtTime(0.0);
  auto obstacle_reference_line_position =
      math::PathMatcher::GetPathFrenetCoordinate(discretized_reference_line,
                                       point.path_point.x,point.path_point.y );

  if (obstacle_reference_line_position.first < ego_vehicle_s - 4.0 &&
      std::fabs(obstacle_reference_line_position.second) < half_lane_width) {
    cout << "Ignore obstacle [" << obstacle->Id() << "]"<<endl;
    return true;
  }
  return false;
}

}  // namespace planning

