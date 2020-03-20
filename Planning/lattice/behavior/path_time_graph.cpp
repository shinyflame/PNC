
#include "./path_time_graph.h"
#include "../../math/path_matcher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>


extern planning::ConfigParam g_config_param;
extern planning::VehicleParam g_vehicle_config;


namespace planning {

PathTimeGraph::PathTimeGraph(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points,
    const ReferenceLineInfo* ptr_reference_line_info,
    const double s_start, const double s_end,
    const double t_start, const double t_end,
    const std::array<double, 3>& init_d ) {
  //CHECK_LT(s_start, s_end);
  //CHECK_LT(t_start, t_end);
  path_range_.first  = s_start;
  path_range_.second = s_end;
  time_range_.first  = t_start;
  time_range_.second = t_end;
  ptr_reference_line_info_ = ptr_reference_line_info;
  init_d_ = init_d;
  SetupObstacles(obstacles, discretized_ref_points);
}


void PathTimeGraph::SetupObstacles(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points)
{
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->IsVirtual()) {//if obstacle is virtual ignore don't set up time_graph
      continue;
    }
    if (!obstacle->HasTrajectory()) {//no predicition Set Static Obstacle
      SetStaticObstacle(obstacle, discretized_ref_points);
    } else {//Set Dynamic Obstacle
      SetDynamicObstacle(obstacle, discretized_ref_points);
    }
  }
  //手工指定小于操作排序
  std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
      [](const SLBoundary& sl0, const SLBoundary& sl1) {
        return sl0.start_s < sl1.start_s;
      });

  // update path_time_obstacles_
  for (auto& path_time_obstacle : path_time_obstacle_map_) {

    double s_upper = std::max(path_time_obstacle.second.bottom_right.s,
                              path_time_obstacle.second.upper_right.s);
    double s_lower = std::min(path_time_obstacle.second.bottom_left.s,
                              path_time_obstacle.second.upper_left.s);
    path_time_obstacle.second.path_lower = (s_lower);
    path_time_obstacle.second.path_upper = (s_upper);

    double t_upper = std::max(path_time_obstacle.second.bottom_right.t,
                              path_time_obstacle.second.upper_right.t);
    double t_lower = std::min(path_time_obstacle.second.bottom_left.t,
                              path_time_obstacle.second.upper_left.t);
    path_time_obstacle.second.time_lower = (t_lower);
    path_time_obstacle.second.time_upper = (t_upper);
    path_time_obstacles_.push_back(path_time_obstacle.second);
  }

}

void PathTimeGraph::SetStaticObstacle( const Obstacle* obstacle,
                          const std::vector<PathPoint>& discretized_ref_points)
{
  //get obstacle geometry polygon
  const Polygon2d& polygon = obstacle->PerceptionPolygon();
  int32_t obstacle_id = obstacle->Id();
  //Compute Obstacle SL Boundary
  SLBoundary sl_boundary = ComputeObstacleBoundary(
                            polygon.GetAllVertices(), discretized_ref_points);

  double left_width = g_config_param.default_reference_line_width * 0.5;
  double right_width = g_config_param.default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      sl_boundary.start_s, left_width, right_width);

  left_width  -= g_config_param.obstacle_filter_buffer;
  right_width -= g_config_param.obstacle_filter_buffer;
  //obstacle out of way range continue next obstacle
  if (sl_boundary.start_s > path_range_.second ||
      sl_boundary.end_s < path_range_.first ||
      sl_boundary.start_l >left_width ||
      sl_boundary.end_l < -right_width) {
    //cout << "Obstacle [" << obstacle_id << "] is out of range."<<endl;
    return;
  }
 //if obstacle in lane draw static obstacle path_time_graph
  path_time_obstacle_map_[obstacle_id].obstacle_id= obstacle_id;

  path_time_obstacle_map_[obstacle_id].bottom_left=
      SetPathTimePoint(obstacle_id, sl_boundary.start_s, 0.0);
  path_time_obstacle_map_[obstacle_id].bottom_right=
      SetPathTimePoint(obstacle_id, sl_boundary.start_s,
                       g_config_param.trajectory_time_length);

  path_time_obstacle_map_[obstacle_id].upper_left=
      SetPathTimePoint(obstacle_id, sl_boundary.end_s, 0.0);
  path_time_obstacle_map_[obstacle_id].upper_right=
      SetPathTimePoint(obstacle_id, sl_boundary.end_s,
                       g_config_param.trajectory_time_length);

  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
  cout << "ST-Graph mapping static obstacle: " << obstacle_id
         << ", start_s : " << sl_boundary.start_s
         << ", end_s : " << sl_boundary.end_s
         << ", start_l : " << sl_boundary.start_l
         << ", end_l : " << sl_boundary.end_l<<endl;
}

void PathTimeGraph::SetDynamicObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points )
{
  double relative_time = time_range_.first;
  while (relative_time < time_range_.second)
  {
    //find relative time corresponding trajectory point
    TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
    //get trajectory point position corresponding box
    Box2d box = obstacle->GetBoundingBox(point);
    //get  car box all corners and sl_boundary
    SLBoundary sl_boundary = ComputeObstacleBoundary(box.GetAllCorners(),
        discretized_ref_points);

    double left_width = g_config_param.default_reference_line_width * 0.5;
    double right_width = g_config_param.default_reference_line_width * 0.5;
    ptr_reference_line_info_->reference_line().GetLaneWidth(
        sl_boundary.start_s, left_width, right_width);

    // the obstacle is not shown on the region to be considered.
    if (sl_boundary.start_s > path_range_.second ||sl_boundary.end_s < path_range_.first
        ||sl_boundary.start_l > left_width ||sl_boundary.end_l < -right_width)
    {
      if (path_time_obstacle_map_.find(obstacle->Id()) !=path_time_obstacle_map_.end()) {
       //if the obstacle  existence in path_time_obstacle_map_ jump out circle
        break;
      } else {
       // the obstacle existence continue next relative time  corresponding trajectory
        relative_time += g_config_param.trajectory_time_resolution;
        continue;
      }
    }

   // the obstacle is shown on the region to be considered.
    if (path_time_obstacle_map_.find(obstacle->Id()) == path_time_obstacle_map_.end())
    {
      //set obstacle id
      path_time_obstacle_map_[obstacle->Id()].obstacle_id=(obstacle->Id());
      //if first time add obstacle set time_graph left value bottom and upper
      path_time_obstacle_map_[obstacle->Id()].bottom_left=
          SetPathTimePoint(obstacle->Id(), sl_boundary.start_s,relative_time);
      path_time_obstacle_map_[obstacle->Id()].upper_left=
          SetPathTimePoint(obstacle->Id(), sl_boundary.end_s,  relative_time);
    }
   //if first time not add obstacle updata time_graph right value bottom and upper
    path_time_obstacle_map_[obstacle->Id()].bottom_right=
        SetPathTimePoint(obstacle->Id(), sl_boundary.start_s, relative_time);
    path_time_obstacle_map_[obstacle->Id()].upper_right=
        SetPathTimePoint(obstacle->Id(), sl_boundary.end_s, relative_time);
    relative_time += g_config_param.trajectory_time_resolution;
  }



}

SLBoundary PathTimeGraph::ComputeObstacleBoundary(
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


PathTimePoint PathTimeGraph::SetPathTimePoint(const int32_t& obstacle_id,
                                              const double s,
                                              const double t) const {
  PathTimePoint path_time_point;
  path_time_point.s=(s);
  path_time_point.t=(t);
  path_time_point.obstacle_id=(obstacle_id);
  return path_time_point;
}

const std::vector<PathTimeObstacle>& PathTimeGraph::GetPathTimeObstacles()
    const {
  return path_time_obstacles_;
}

bool PathTimeGraph::GetPathTimeObstacle(const int32_t& obstacle_id,
                                        PathTimeObstacle* path_time_obstacle) {
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return false;
  }
  *path_time_obstacle = path_time_obstacle_map_[obstacle_id];
  return true;
}

std::vector<std::pair<double, double>>
PathTimeGraph::GetPathBlockingIntervals(const double t) const
{
  std::vector<std::pair<double, double>> intervals;
  if(t  < time_range_.first  || t > time_range_.second)
    {
      std::cout<<"ERROR!!! t out of time_range_"<<std::endl;
      return intervals; }

  for (const auto& pt_obstacle : path_time_obstacles_)
  {
    if (t > pt_obstacle.time_upper || t < pt_obstacle.time_lower) {
      continue;
    }
    double s_upper =
        math::lerp(pt_obstacle.upper_left.s,  pt_obstacle.upper_left.t,
                   pt_obstacle.upper_right.s, pt_obstacle.upper_right.t, t);
    double s_lower =
        math::lerp(pt_obstacle.bottom_left.s, pt_obstacle.bottom_left.t,
             pt_obstacle.bottom_right.s, pt_obstacle.bottom_right.t, t);
   std::pair<double, double> interval(s_lower, s_upper);
    intervals.emplace_back(interval);
  }
  return intervals;
}

std::vector<std::vector<std::pair<double, double>>>
PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                        const double t_end,
                                        const double t_resolution) {
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = t_start; t <= t_end; t += t_resolution)
  {
    if (GetPathBlockingIntervals(t).size() >= 0)
     {
        intervals.push_back(GetPathBlockingIntervals(t));
     }else{
        //cout<<"t = "<<t<<", GetPathBlockingIntervals(t).size() < 0"<<endl;
        continue;
     }
  }
  return intervals;
}

std::pair<double, double> PathTimeGraph::get_path_range() const {
  return path_range_;
}

std::pair<double, double> PathTimeGraph::get_time_range() const {
  return time_range_;
}

std::vector<PathTimePoint> PathTimeGraph::GetObstacleSurroundingPoints(
     const int32_t& obstacle_id, const double s_dist,const double t_min_density) const
{
  std::vector<PathTimePoint> pt_pairs;

  if(t_min_density <= 0) {
      std::cout<<"ERROR t_min_density <= 0"<<endl;
      return pt_pairs;
    }
  if (path_time_obstacle_map_.find(obstacle_id) ==
      path_time_obstacle_map_.end()) {
    return pt_pairs;
  }

  const auto& pt_obstacle = path_time_obstacle_map_.at(obstacle_id);
  double s0 = 0.0;
  double s1 = 0.0;
  double t0 = 0.0;
  double t1 = 0.0;

  if (s_dist > 0.0) {//overide car
    s0 = pt_obstacle.upper_left.s;
    s1 = pt_obstacle.upper_right.s;
    t0 = pt_obstacle.upper_left.t;
    t1 = pt_obstacle.upper_right.t;
  } else { //follow car
    s0 = pt_obstacle.bottom_left.s;
    s1 = pt_obstacle.bottom_right.s;
    t0 = pt_obstacle.bottom_left.t;
    t1 = pt_obstacle.bottom_right.t;
  }

  double time_gap = t1 - t0;//path time graph time gap
  //CHECK(time_gap > -g_config_param.lattice_epsilon);
  time_gap = std::fabs(time_gap);

  std::size_t num_sections = std::size_t(time_gap / t_min_density) + 1;
  double t_interval = time_gap / num_sections;

  for (std::size_t i = 0; i <= num_sections; ++i) {
    double t = t_interval * i + t0;
    double s = math::lerp(s0, t0, s1, t1, t) + s_dist;

    PathTimePoint ptt;
    ptt.obstacle_id=(obstacle_id);
    ptt.t = t ;
    ptt.s = s ;
    pt_pairs.push_back(std::move(ptt));
  }

  return pt_pairs;
}

bool PathTimeGraph::IsObstacleInGraph(const int32_t& obstacle_id) {
  return path_time_obstacle_map_.find(obstacle_id) !=
         path_time_obstacle_map_.end();
}
//no using
std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(
    const double s_start, const double s_end, const double s_resolution) {
  //CHECK_LT(s_start, s_end);
  //CHECK_GT(s_resolution, g_config_param.lattice_epsilon);
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;
  double s_range = s_end - s_start;
  double s_curr = s_start;
  std::size_t num_bound = static_cast<std::size_t>(s_range / s_resolution);

  const auto& vehicle_config = g_vehicle_config;
  double ego_width = vehicle_config.width;

  // Initialize bounds by reference line width
  for (std::size_t i = 0; i < num_bound; ++i)
  {
    double left_width = g_config_param.default_reference_line_width / 2.0;
    double right_width = g_config_param.default_reference_line_width / 2.0;
    ptr_reference_line_info_->reference_line().GetLaneWidth(
                                        s_curr, left_width, right_width);
    //compute lane width edge whit car_width from max_L to min_L
    double ego_d_lower = init_d_[0] - ego_width / 2.0;
    double ego_d_upper = init_d_[0] + ego_width / 2.0;
    std::pair<double, double> bound(//bound_buffer = 0.1
          std::min(-right_width, ego_d_lower - g_config_param.bound_buffer),
          std::max( left_width,  ego_d_upper + g_config_param.bound_buffer) );
    bounds.emplace_back(bound);
    discretized_path.push_back(s_curr);
    s_curr += s_resolution;
  }

  for (const SLBoundary& static_sl_boundary : static_obs_sl_boundaries_) {
    UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path,
                                               s_start, s_end, &bounds);
  }

  for (std::size_t i = 0; i < bounds.size(); ++i) {
    bounds[i].first += ego_width / 2.0;
    bounds[i].second -= ego_width / 2.0;
    if (bounds[i].first >= bounds[i].second) {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }
  return bounds;
}
// no using
void PathTimeGraph::UpdateLateralBoundsByObstacle(
                     const SLBoundary& sl_boundary,
                     const std::vector<double>& discretized_path,
                     const double s_start, const double s_end,
                     std::vector<std::pair<double, double>>* const bounds)
{
  if (sl_boundary.start_s > s_end || sl_boundary.end_s < s_start) {
    return;//judge obstacle is in between s_start and s_end
  }
  auto start_iter = std::lower_bound(// 第一个   >= sl_boundary.start_s
       discretized_path.begin(), discretized_path.end(), sl_boundary.start_s);
  auto end_iter = std::upper_bound(  // 第一个   >  sl_boundary.start_s
       discretized_path.begin(), discretized_path.end(), sl_boundary.start_s);
      
  std::size_t start_index = start_iter - discretized_path.begin();
  std::size_t end_index   = end_iter   - discretized_path.begin();
  //if min_l > 0.00001 and max_l < -0.00001
  if (sl_boundary.end_l   > -g_config_param.lattice_epsilon &&
      sl_boundary.start_l <  g_config_param.lattice_epsilon )
  {
    for (std::size_t i = start_index; i < end_index; ++i)
    {
      bounds->operator[](i).first = -g_config_param.lattice_epsilon;
      bounds->operator[](i).second = g_config_param.lattice_epsilon;
    }
    return;
  }
  //if min_l < 0.0
  if (sl_boundary.end_l < g_config_param.lattice_epsilon)
  {
    for (std::size_t i = start_index; i < std::min(end_index + 1, bounds->size()); ++i)
    {
      bounds->operator[](i).first =
         std::max(bounds->operator[](i).first,
                   sl_boundary.end_l + g_config_param.nudge_buffer);
    }
    return;
  }
  //if max_l > 0.0
  if (sl_boundary.start_l > -g_config_param.lattice_epsilon)
  {
    for (std::size_t i = start_index;
         i < std::min(end_index + 1, bounds->size()); ++i)
    {
      bounds->operator[](i).second = std::min( bounds->operator[](i).second,
                          sl_boundary.start_l - g_config_param.nudge_buffer);
    }
    return;
  }
}

} //end namespace planning
