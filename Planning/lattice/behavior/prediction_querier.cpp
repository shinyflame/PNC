
/**
 * @file
 **/

#include <string>
#include "../../math/linear_interpolation.h"
#include "../../math/path_matcher.h"
#include "./prediction_querier.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {



PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    const std::shared_ptr<std::vector<PathPoint>>& ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line)
{
  // add each obstacle to id_obstacle_map_
  for (const auto ptr_obstacle : obstacles)
  {
    if (id_obstacle_map_.find(ptr_obstacle->Id()) == id_obstacle_map_.end())
    {
      id_obstacle_map_[ptr_obstacle->Id()] = ptr_obstacle;
      obstacles_.push_back(ptr_obstacle);
    } else {
      cout<<"AWARN Duplicated obstacle found ["<< ptr_obstacle->Id()<<"]"<<endl;
    }
  }
}

std::vector<const Obstacle*> PredictionQuerier::GetObstacles() const {
  return obstacles_;
}

//动态障碍物在参考线方向上的速度投影
double PredictionQuerier::ProjectVelocityAlongReferenceLine(
     const int32_t& obstacle_id, const double s, const double t) const {
 // CHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());
 if(id_obstacle_map_.find(obstacle_id) == id_obstacle_map_.end())
   {
     cout<<"Error in id_obstacle_map not find obstacle_id "<<endl;
     return 0.0;
   }

  const prediction::Trajectory&
      trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();
  int num_traj_point = trajectory.trajectory_point.size();
  if (num_traj_point < 2) {//静态障碍物
    return 0.0;
  }

  if (t < trajectory.trajectory_point[0].relative_time ||//时间点不再预测范围
      t > trajectory.trajectory_point[num_traj_point - 1].relative_time) {
    return 0.0;
  }

  auto matched_it = std::lower_bound(trajectory.trajectory_point.begin(),
      trajectory.trajectory_point.end(), t,
      [](const TrajectoryPoint& p, const double t)
      { return p.relative_time < t; });//找到first >=t的trajectory point中的那个

  double v = matched_it->v;
  double theta = matched_it->path_point.theta;
  double v_x = v * std::cos(theta + M_PI/2.0);
  double v_y = v * std::sin(theta + M_PI/2.0) ;//分解成x y方向的速度
  //int size = g_config_param.segment_length / g_config_param.discretion_length;
  int self_index = s /  g_config_param.discretion_length;
  int start_index = self_index -10;
  if(start_index < 0) start_index =0;
  int end_index = self_index +10;
  if(end_index > ptr_reference_line_.get()->size() -1)
      end_index = ptr_reference_line_.get()->size() - 1;
  PathPoint obstacle_point_on_ref_line =
                math::PathMatcher::MatchToPath(*ptr_reference_line_, s,start_index,end_index);
  auto ref_theta = obstacle_point_on_ref_line.theta + M_PI/2.0;
  //参考线方向上速度的投影
  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;
}

} //end namespace planning
