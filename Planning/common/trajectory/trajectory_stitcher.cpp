
/**
 * @file
 **/

#include <algorithm>
#include <list>
#include <utility>

#include "../../common/trajectory/trajectory_stitcher.h"
#include "../../math/angle.h"
#include "../../math/quaternion.h"
#include "../../math/util.h"


extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {

std::vector<TrajectoryPoint> TrajectoryStitcher::
ComputeReinitStitchingTrajectory(const VehicleState& vehicle_state)
{
  TrajectoryPoint init_point;
  init_point.path_point.s = (0.0);
  init_point.path_point.x = (vehicle_state.x);
  init_point.path_point.y = (vehicle_state.y);
  init_point.path_point.z = (vehicle_state.z);
  init_point.path_point.theta = ( vehicle_state.heading);
  init_point.path_point.kappa = (vehicle_state.kappa);
  init_point.v=(vehicle_state.linear_velocity);
  init_point.a=(vehicle_state.linear_acceleration);
  init_point.relative_time=(0.0);
  cout<<"Note using vehicle status as planning initial point !!!"<<endl;
  return std::vector<TrajectoryPoint>(1, init_point);
}

// only used in navigation mode
void TrajectoryStitcher::
TransformLastPublishedTrajectory(const double x_diff, const double y_diff,
                                 const double theta_diff,
                                 PublishableTrajectory* prev_trajectory)
{
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->trajectory_points().begin(),
                prev_trajectory->trajectory_points().end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](TrajectoryPoint& p) {
                  auto x = p.path_point.x;
                  auto y = p.path_point.y;
                  auto theta = p.path_point.theta;

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = WrapAngle(theta - theta_diff);

                  p.path_point.x = (x_new);
                  p.path_point.y = (y_new);
                  p.path_point.theta = (theta_new);
                });
}

// Planning from current vehicle state:
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<TrajectoryPoint> TrajectoryStitcher::
ComputeStitchingTrajectory(  const VehicleState& vehicle_state,
                             const SLPoint &adc_sl,
                             const uint64_t current_timestamp,
                             const double planning_cycle_time,
                             const PublishableTrajectory* prev_trajectory )
{
  if (!g_config_param.enable_trajectory_stitcher) {
    cout<<"Note disable_trajectory_stitcher"<<endl;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (!prev_trajectory) {
    cout<<"Note don't have prev_trajectory"<<endl;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  if(abs(adc_sl.l) > 1.5 * g_config_param.default_reference_line_width){
      cout<<"Note adc L > default_reference_line_width !"<<endl;
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
  static int count_mannul_drive_times = 0;
  if (vehicle_state.driving_mode != COMPLETE_AUTO_DRIVE) {
    count_mannul_drive_times++;
  }else{
    count_mannul_drive_times = 0;
  }

  if(count_mannul_drive_times > 3 ){
    cout<<"Note driving_mode != COMPLETE_AUTO_DRIVE"<<endl;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
//  if (vehicle_state.linear_velocity < g_config_param.max_stop_speed / 3.0) {
//    cout<<"linear_velocity too mini !!!"<<endl;
//    return ComputeReinitStitchingTrajectory(vehicle_state);
//  }

  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
   cout << "Projected trajectory at time [" << prev_trajectory->header_time()
        << "] size is zero! Previous planning not exist or failed. Use "
        <<  "origin car status instead."<<endl;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  const double veh_rel_time =
                (current_timestamp - prev_trajectory->header_time()) / 1000.0;

  std::size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);
#if 1

//  if (time_matched_index == 0 &&
//      veh_rel_time < prev_trajectory->StartPoint().relative_time) {
//    cout << "current time smaller than the "
//            "previous trajectory's first time"<<endl;
//    return ComputeReinitStitchingTrajectory(vehicle_state);
//  }

//  if (time_matched_index + 1 >= prev_trajectory_size) {
//    cout<< "current time beyond the previous trajectory's last time"<<endl;
//    return ComputeReinitStitchingTrajectory(vehicle_state);
//  }

//  auto time_matched_point =
//      prev_trajectory->TrajectoryPointAt(time_matched_index);

//  if ( &time_matched_point == NULL) {//modify by sxl @2018.12.20
//    cout<<"Info time_matched_point == NULL"<<endl;
//    return ComputeReinitStitchingTrajectory(vehicle_state);
//  }
#endif
  std::size_t position_matched_index = prev_trajectory->QueryNearestPoint(
      {vehicle_state.x, vehicle_state.y});

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x, vehicle_state.y,
      prev_trajectory->TrajectoryPointAt(position_matched_index));

  //auto lon_diff = time_matched_point.path_point.s - frenet_sd.first;
  auto lat_diff = frenet_sd.second;

//  cout << "Control lateral diff: " << lat_diff
//       << ", longitudinal diff: " << lon_diff<<endl;
#if 0
  if (std::fabs(lat_diff) > g_config_param.replan_lateral_distance_threshold ||
      //replan_lateral_distance_threshold = 5
      std::fabs(lon_diff) > g_config_param.replan_longitudinal_distance_threshold) {
      //replan_longitudinal_distance_threshold = 5
   cout << "the distance between matched point and actual position is too "
        << "large. Replan is triggered. lat_diff = "
        << lat_diff << ", lon_diff = " << lon_diff<<endl;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
#endif
  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(time_matched_index).relative_time +
      planning_cycle_time;

  std::size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  cout << "Position matched index: " << position_matched_index<<endl;
  cout << "Time matched index: " << time_matched_index<<endl;

  //auto matched_index = std::min(time_matched_index, position_matched_index);
  //if(abs(time_matched_index - position_matched_index) > 5 )
   int matched_index = position_matched_index;
//  std::vector<TrajectoryPoint> stitching_trajectory(
//      prev_trajectory->trajectory_points().begin()
//        +std::max(0, static_cast<int>(matched_index - 1)),
//      prev_trajectory->trajectory_points().begin()
//        + forward_time_index + 1  );
  int size =  prev_trajectory->trajectory_points().size()-1;
  int behind_index = max(matched_index-1, 0 );
  int front_index = max(matched_index-1, 0) +
                    g_config_param.prediction_point_num;
  if(front_index > size)
      front_index = size;

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->trajectory_points().begin()+ behind_index,
      prev_trajectory->trajectory_points().begin()+ front_index );

  const double zero_s = stitching_trajectory.back().path_point.s;
  const double time_gap = (current_timestamp - prev_trajectory->header_time())/1000.0 ;
  for (auto& tp : stitching_trajectory) {
    if (&tp == NULL) {//modify bu sxl @ 2018.12.20
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.relative_time= tp.relative_time - time_gap;
    tp.path_point.s = tp.path_point.s  - zero_s;
    //tp.
  }
  cout<<"stitching_trajectory veh_rel_time = "<<veh_rel_time<<endl;
  cout<<"stitching_trajectory time_matched_index = "<<time_matched_index<<endl;
  cout<<"stitching_trajectory stitching_trajectory.size() = "<<stitching_trajectory.size()<<endl;
  //current_timestamp - prev_trajectory->header_time()
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::
ComputePositionProjection(const double x, const double y, const TrajectoryPoint& p)
{
  Vec2d v(x - p.path_point.x, y - p.path_point.y);

  Vec2d n( cos(p.path_point.theta + M_PI/2.0),
           sin(p.path_point.theta + M_PI/2.0) );

  std::pair<double, double> frenet_sd;
  frenet_sd.first  = v.InnerProd(n) + p.path_point.s;
  frenet_sd.second = v.CrossProd(n);
  cout<<"ComputePositionProjection p.path_point.s= "<<p.path_point.s <<endl;
  cout<<"ComputePositionProjection v.InnerProd(n)= "<<v.InnerProd(n) <<endl;
  return frenet_sd;
}

}  // namespace planning

