
/**
 * @file
 **/

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "../../math/path_matcher.h"
#include "../../lattice/trajectory_generation/trajectory_evaluator.h"
#include "../../constraint_checker/constraint_checker1d.h"
#include "../../lattice/trajectory1d/piecewise_acceleration_trajectory1d.h"
#include "../../lattice/trajectory_generation/piecewise_braking_trajectory_generator.h"
#include "../../common/get_now_time.h"

extern ConfigParam g_config_param;


namespace planning {
double t[7];

using Trajectory1d = Curve1d;
using Trajectory1dPair =
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>;
using CostComponentsPair = std::pair<std::vector<double>, double>;

using PtrTrajectory1d = std::shared_ptr<Trajectory1d>;

TrajectoryEvaluator::TrajectoryEvaluator(
    const std::array<double, 3>& init_s,
    const std::array<double, 3>& init_d,
    const PlanningTarget& planning_target,
    const std::vector<PtrTrajectory1d>& lon_trajectories,
    const std::vector<PtrTrajectory1d>& lat_trajectories,
    std::shared_ptr<PathTimeGraph> path_time_graph,
    std::shared_ptr<std::vector<PathPoint>> reference_line,
    const vector<SLPoint>& garbages_sl,
    const bool is_slide_clean)
    : path_time_graph_(path_time_graph),
      reference_line_(reference_line),
      init_s_(init_s),
      init_d_(init_d),
      garbages_sl_(garbages_sl),is_slide_clean_(is_slide_clean)
{
  const double start_time = 0.0;
  const double end_time = g_config_param.trajectory_time_length;
  path_time_intervals_  = path_time_graph_->GetPathBlockingIntervals(
      start_time, end_time, g_config_param.trajectory_time_resolution);

  reference_s_dot_ = ComputeLongitudinalGuideVelocity(planning_target);

  // if we have a stop point along the reference line,
  // filter out the lon. trajectories that pass the stop point.
  double stop_point_s = std::numeric_limits<double>::max();
  if (planning_target.has_stop_point) {
    stop_point_s = planning_target.stop_point.s;
  }
  //init_[0]------------------stop_point------------------------lon_end_s
  for(int i = 0;i <7; i++)
    {
      t[i] = 0.0;
    }

  for (const auto& lon_trajectory : lon_trajectories)
  {
    double lon_end_s = lon_trajectory->Evaluate(0, end_time);
    if (init_s[0] < stop_point_s + g_config_param.lattice_stop_buffer + 1&&
        lon_end_s > stop_point_s + g_config_param.lattice_stop_buffer)
     { continue;  }
    // filter lon_trajectory dosen't fit the vehicle kinetic characteristic
    if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*lon_trajectory))
      {  continue;  }
    std::vector<double> cost_components;
    double lon_cost = LonCostEvalute(planning_target, lon_trajectory,&cost_components );


    for (const auto& lat_trajectory : lat_trajectories)
     {
//      The validity of the code needs to be verified.
//      if (!ConstraintChecker1d::
//           IsValidLateralTrajectory(*lat_trajectory,*lon_trajectory,init_d_[0]))
//        { continue; }

        double lat_cost = LatCostEvalute( lon_trajectory,lat_trajectory, &cost_components);
        double cost = lon_cost + lat_cost;
        cost_queue_with_components_.emplace( Trajectory1dPair(lon_trajectory, lat_trajectory),
                                           CostComponentsPair(cost_components, cost));
        cost_queue_.emplace(Trajectory1dPair(lon_trajectory, lat_trajectory),cost);
     }
  }

  for(int i = 0;i <7; i++)
    {
      cout<<i+1<<" cost_time_ms = "<<t[i]*1000<<endl;
    }

  if (!g_config_param.enable_auto_tuning) {
   cout << "Number of valid 1d trajectory pairs: " << cost_queue_.size()<<endl;
  } else {
   cout << "Number of valid 1d trajectory pairs: "
        << cost_queue_with_components_.size()<<endl;
  }
}

double TrajectoryEvaluator::LonCostEvalute(const PlanningTarget& planning_target,
                        const std::shared_ptr<Curve1d>& lon_trajectory,
                        std::vector<double>* cost_components) const{

  // Lon costs
  // 1.lon_objective_cost
  // 2.lon_jerk_cost
  // 3.lon_collision_cost
  // 4.centripetal_acc_cost
  double tt = Clock::NowInSeconds();
  double lon_objective_cost =
           LonObjectiveCost(lon_trajectory, planning_target, reference_s_dot_);
  t[0] += Clock::NowInSeconds() - tt;

  tt = Clock::NowInSeconds();
  double lon_jerk_cost = LonComfortCost(lon_trajectory);
  t[1] += Clock::NowInSeconds() - tt;

  tt = Clock::NowInSeconds();
  double lon_collision_cost = LonCollisionCost(lon_trajectory);
  t[2] += Clock::NowInSeconds() - tt;

  tt = Clock::NowInSeconds();
  double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);
  //cout<<"centripetal_acc_cost = " <<centripetal_acc_cost<<endl;
  t[3] += Clock::NowInSeconds() - tt;

  if(cost_components != nullptr) {
      cost_components->emplace_back(lon_objective_cost );
      cost_components->emplace_back(lon_jerk_cost );
      cost_components->emplace_back(lon_collision_cost );
      cost_components->emplace_back(centripetal_acc_cost );
  }

  return lon_objective_cost   * g_config_param.weight_lon_objective +          // 10.0
         lon_jerk_cost        * g_config_param.weight_lon_jerk +               // 1.0
         lon_collision_cost   * g_config_param.weight_lon_collision+           // 5.0
         centripetal_acc_cost * g_config_param.weight_centripetal_acceleration //1.5
         ;
}

double TrajectoryEvaluator::LatCostEvalute(
                        const std::shared_ptr<Curve1d>& lon_trajectory,
                        const std::shared_ptr<Curve1d>& lat_trajectory,
                        std::vector<double>* cost_components) const{
  // Lat Costs:
  // 5. Cost of lateral offsets
  // 6. Cost of lateral comfort
  // 7. Cost of clean complete
  double lat_offset_cost = 0;
  double clean_complete_cost = 0 , tt = 0;

  tt = Clock::NowInSeconds();
  double evaluation_horizon =
      std::min(g_config_param.decision_horizon,
               lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()) -
               lon_trajectory->Evaluate(0, 0.0) );

  vector <double> s_values1;
  for (double s = 0.0; s < evaluation_horizon;s += 0.5) {// 0.1m
    s_values1.emplace_back(s);
  }

  double s0 = lon_trajectory->Evaluate(0,0.0);
  vector<double> lat_offset_s;
  for(double t = 0.0;t < g_config_param.trajectory_time_length;
             t += 3*g_config_param.trajectory_time_resolution ) {
      lat_offset_s.push_back(lon_trajectory->Evaluate(0,t) - s0);
  }

  lat_offset_cost = LatOffsetCost(lat_trajectory, lat_offset_s);
  t[4] += Clock::NowInSeconds() - tt;

  tt = Clock::NowInSeconds();
  double lat_comfort_cost = LatComfortCost(lon_trajectory, lat_trajectory);
 // cout<<"lat_comfort_cost = "<<lat_comfort_cost<<endl;
  t[5] += Clock::NowInSeconds() - tt;

  tt = Clock::NowInSeconds();
  clean_complete_cost = CleanCompleteCost(lat_trajectory,garbages_sl_,s0,lat_offset_s);
  //cout<<"clean_complete_cost = "<<clean_complete_cost<<endl;
  t[6] += Clock::NowInSeconds() - tt;

  if(cost_components != nullptr) {
      cost_components->emplace_back(lat_offset_cost );
      cost_components->emplace_back(lat_comfort_cost );
      cost_components->emplace_back(clean_complete_cost);

  }

  return lat_offset_cost      * g_config_param.weight_lat_offset +     // 2.0
         lat_comfort_cost     * g_config_param.weight_lat_comfort+     // 10.0
         clean_complete_cost  * g_config_param.weight_clean_garbage;
}




double TrajectoryEvaluator::LatOffsetCost(const PtrTrajectory1d& lat_trajectory,
                                          const std::vector<double>& s_values) const
{
  double lat_offset_start = lat_trajectory->Evaluate(0, 0.0);
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  double coefficient = 1;


  for ( auto s : s_values)
  {

    double lat_offset = lat_trajectory->Evaluate(0, s);
    lat_offset -= g_config_param.slide_trans_dis;
    if(is_slide_clean_)
      if(lat_offset < g_config_param.slide_trans_dis){
          double gap = g_config_param.slide_trans_dis - lat_offset;
          if(gap > g_config_param.max_slide_gap)
              gap = g_config_param.max_slide_gap;
          double coef  =  gap/g_config_param.max_slide_gap;
          coefficient +=  coef*coef;
     }

    lat_offset *= coefficient;
    double abs_lat_offset = abs(lat_offset );
    if( abs_lat_offset > g_config_param.lat_offset_bound){
        abs_lat_offset = g_config_param.lat_offset_bound;
     }
    double cost = abs_lat_offset  / g_config_param.lat_offset_bound;//3.0 lat_offset - max= 1
    //cost_abs_sum += std::fabs(cost);
    if (lat_offset * lat_offset_start < 0.0) {// original is < 0.0 ? 2019.04.16 modify by sxl
      //cost_sqr_sum += cost * cost * g_config_param.weight_opposite_side_offset;//10.0
      cost_abs_sum +=        cost * g_config_param.weight_opposite_side_offset ;
    } else {
      //cost_sqr_sum += cost * cost * g_config_param.weight_same_side_offset;//1.0
      cost_abs_sum +=        cost * g_config_param.weight_same_side_offset ;

    }
  }

  //cout <<"cost = "<<cost_sqr_sum / (cost_abs_sum + g_config_param.lattice_epsilon)<<endl;
  //return cost_sqr_sum / (cost_abs_sum + g_config_param.lattice_epsilon);

  return cost_abs_sum / (s_values.size() + g_config_param.lattice_epsilon);
}



double TrajectoryEvaluator::LatComfortCost(
                                 const PtrTrajectory1d& lon_trajectory,
                                 const PtrTrajectory1d& lat_trajectory) const
{
  double s_dot = 0.0,s_dotdot = 0.0;
  double cost_sqr_sum = 0.0,cost_abs_sum =0.0,comfort_cost_ = 0;
  double cost = 0;
  double s0 = lon_trajectory->Evaluate(0, 0.0);

  for (double t = 0.0; t < lon_trajectory->ParamLength();//g_config_param.trajectory_time_length;
                       t += 2*g_config_param.trajectory_time_resolution)
  {

    double s  = lon_trajectory->Evaluate(0, t);
    if (s > lat_trajectory->ParamLength() + s0){//S value > /4/8/12/16
        break;
      }

    double theta_s = s - s0;
    s_dot    = lon_trajectory->Evaluate(1, t);
    s_dotdot = lon_trajectory->Evaluate(2, t);

    double dl_ = lat_trajectory->Evaluate(1, theta_s);
    double d2l_= lat_trajectory->Evaluate(2, theta_s);
    cost = abs(d2l_ * s_dot * s_dot + dl_ * s_dotdot);
    if(cost > g_config_param.lateral_acceleration_bound){
        cost = g_config_param.lateral_acceleration_bound;
    }

    cost /= g_config_param.lateral_acceleration_bound;
    cost_sqr_sum += cost*cost;
    cost_abs_sum += cost;
  }

  comfort_cost_ = cost_sqr_sum/(cost_abs_sum + g_config_param.lattice_epsilon);
  return comfort_cost_;
}

double TrajectoryEvaluator::CleanCompleteCost(
                         const std::shared_ptr<Curve1d>& lat_trajectory,
                         const vector<SLPoint>& sl_points,
                         const double &s0,
                         const vector<double> &s_value)const
{


  double sum_square_cost = 0, sum_abs_cost = 0;
  double clean_max_evaluate_width = g_config_param.clean_max_evaluate_width;
  double clean_ignore_dis = g_config_param.clean_ignore_dis;
  double clean_effect_dis = g_config_param.clean_effect_dis;

  for(auto sl_point :sl_points)
  {
    if(sl_point.s < s0 + clean_ignore_dis  || sl_point.s > s0 + clean_effect_dis) continue;
    double s = sl_point.s - s0;
    if(s > lat_trajectory->ParamLength())
        break;//s = lat_trajectory->ParamLength();

    double l = lat_trajectory->Evaluate(0,s);
    double theta_l = abs(l - sl_point.l);
    if( theta_l > clean_max_evaluate_width) continue;
    double dis_coffcent = pow((1-(s-clean_ignore_dis)/clean_effect_dis),2);
    sum_square_cost +=   pow((1-theta_l /clean_max_evaluate_width),2)*0.22;
    sum_abs_cost +=  (1 - theta_l/clean_max_evaluate_width)*dis_coffcent ;
  }

  return  1- sum_square_cost;//sum_square_cost/(sum_abs_cost+ g_config_param.lattice_epsilon);

}




double TrajectoryEvaluator::
LonComfortCost( const PtrTrajectory1d& lon_trajectory ) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (double t = 0.0; t < lon_trajectory->ParamLength();//g_config_param.trajectory_time_length;
       t += g_config_param.trajectory_time_resolution) {
    double jerk = lon_trajectory->Evaluate(3, t);
    double cost = jerk / g_config_param.longitudinal_jerk_upper_bound;
    cost_sqr_sum += cost * cost;
    cost_abs_sum += std::fabs(cost);
  }
  return cost_sqr_sum / (cost_abs_sum + g_config_param.lattice_epsilon);
}


double TrajectoryEvaluator::LonObjectiveCost(
    const PtrTrajectory1d& lon_trajectory,
    const PlanningTarget& planning_target,
    const std::vector<double>& ref_s_dots) const
{
  double t_max  = lon_trajectory->ParamLength();
  double dist_s =
      lon_trajectory->Evaluate(0, t_max) - lon_trajectory->Evaluate(0, 0.0);

  double speed_cost_sqr_sum = 0.0;
  double speed_cost_weight_sum = 0.0;
  for (std::size_t i = 0; i < ref_s_dots.size(); ++i)
  {
    double t = i * g_config_param.trajectory_time_resolution;
    double cost = ref_s_dots[i] - lon_trajectory->Evaluate(1, t);
    speed_cost_sqr_sum += t * t * std::fabs(cost);
    speed_cost_weight_sum += t * t;
  }
  double speed_cost =
   speed_cost_sqr_sum / (speed_cost_weight_sum + g_config_param.lattice_epsilon);
  double dist_travelled_cost = 1.0 / (1.0 + dist_s);

  ///add by sxl @20190523
  if(planning_target.has_stop_point){
      double theta_s  =
      abs(planning_target.stop_point.s - lon_trajectory->Evaluate(0, t_max));
      double theta_S  =
      abs(planning_target.stop_point.s - lon_trajectory->Evaluate(0, 0.0));
      dist_travelled_cost += theta_s /( theta_S +1);
    }
  ///add end
   //cout<<"speed_cost = "<<speed_cost<<endl;
  return (speed_cost * g_config_param.weight_target_speed +
          dist_travelled_cost * g_config_param.weight_dist_travelled) /
         (g_config_param.weight_target_speed + g_config_param.weight_dist_travelled);
}

// TODO(all): consider putting pointer of reference_line_info and frame
// while constructing trajectory evaluator
double TrajectoryEvaluator::
LonCollisionCost(const PtrTrajectory1d& lon_trajectory) const
{
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < path_time_intervals_.size(); ++i) {
    const auto& pt_interval = path_time_intervals_[i];//obstalces head & bottom
    if (pt_interval.empty()) {
      continue;
    }
    double t = i * g_config_param.trajectory_time_resolution;
    double traj_s = lon_trajectory->Evaluate(0, t);
    double sigma = g_config_param.lon_collision_cost_std;//0.5
    for (const auto& m : pt_interval)
    {
      double dist = 0.0;
      if (traj_s < m.first - g_config_param.lon_collision_yield_buffer) {//1.0m
        dist = m.first - traj_s- g_config_param.lon_collision_yield_buffer ;
      } else if (traj_s > m.second + g_config_param.lon_collision_overtake_buffer){//5.0m
        dist = traj_s - m.second - g_config_param.lon_collision_overtake_buffer;
      }
      double cost = std::exp(-dist * dist / (2.0 * sigma * sigma));//e^n

      cost_sqr_sum += cost * cost;
      cost_abs_sum += cost;
    }
  }
  return cost_sqr_sum / (cost_abs_sum + g_config_param.lattice_epsilon);
}


//in this file don't using the function
double TrajectoryEvaluator::LonCollisionCost(
    const std::vector<SpeedPoint>& st_points) const {
  double cost_sqr_sum = 0.0;
  double cost_abs_sum = 0.0;
  for (std::size_t i = 0; i < path_time_intervals_.size(); ++i) {
    const auto& pt_interval = path_time_intervals_[i];
    if (pt_interval.empty()) {
      continue;
    }
    double t = i * g_config_param.trajectory_time_resolution;
    double traj_s = std::numeric_limits<double>::infinity();
    if (!InterpolateDenseStPoints(st_points, t, &traj_s)) {
      cout << "AutoTuning LonCollisionCost InterpolateDenseStPoints Error"<<endl;
      return traj_s;
    }
    double sigma = g_config_param.lon_collision_cost_std;
    for (const auto& m : pt_interval) {
      double cost = 0.0;
      if (traj_s > m.first - g_config_param.lon_collision_yield_buffer &&
          traj_s < m.second + g_config_param.lon_collision_overtake_buffer) {
        cost = 1.0;
      } else if (traj_s < m.first) {
        double dist = (m.first - g_config_param.lon_collision_yield_buffer) - traj_s;
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      } else if (traj_s > m.second) {
        double dist = traj_s - (m.second + g_config_param.lon_collision_overtake_buffer);
        cost = std::exp(-dist * dist / (2.0 * sigma * sigma));
      }
      cost_sqr_sum += cost * cost;
      cost_abs_sum += std::fabs(cost);
    }
  }
  return cost_sqr_sum / (cost_abs_sum + g_config_param.lattice_epsilon);
}

double TrajectoryEvaluator::CentripetalAccelerationCost(
    const PtrTrajectory1d& lon_trajectory) const {

  // Assumes the vehicle is not obviously deviate from the reference line.
  double centripetal_acc_sum = 0.0;
  double centripetal_acc_sqr_sum = 0.0;
  double s0 = lon_trajectory->Evaluate(0, 0);
  double s_max = lon_trajectory->Evaluate(0, 8.0);
  for (double t = 0; t < g_config_param.trajectory_time_length;
                     t += 2.0 * g_config_param.trajectory_time_resolution) {
    double s = lon_trajectory->Evaluate(0, t);
    double v = lon_trajectory->Evaluate(1, t);

    int self_index = s /  g_config_param.discretion_length;
    int start_index = self_index - 10;
    if(start_index < 0) start_index =0;
    int end_index = self_index + 10;
    if(end_index > reference_line_.get()->size() -1)
        end_index = reference_line_.get()->size() - 1;
    PathPoint ref_point =
    math::PathMatcher::MatchToPath(*reference_line_, s,start_index,end_index);

    v *= 4.0;
    double centripetal_acc = std::fabs(v * v * ref_point.kappa);
    //cout<<"centripetal_acc = "<<centripetal_acc<<endl;
    if(centripetal_acc > g_config_param.lateral_acceleration_bound)
        centripetal_acc = g_config_param.lateral_acceleration_bound;
    double cost = centripetal_acc / (g_config_param.lateral_acceleration_bound + 0.1);
    centripetal_acc_sum += cost;
    centripetal_acc_sqr_sum += cost * cost;
  }

  return centripetal_acc_sqr_sum /
         (centripetal_acc_sum + g_config_param.lattice_epsilon);
}


// compute reference velocity for evaluator longitudianl bund
std::vector<double> TrajectoryEvaluator::
ComputeLongitudinalGuideVelocity(const PlanningTarget& planning_target) const
{
  std::vector<double> reference_s_dot;
  double cruise_v = planning_target.cruise_speed ;
  // 1. case one don't have stop point
  if (!planning_target.has_stop_point) {
    PiecewiseAccelerationTrajectory1d lon_traj(init_s_[0], cruise_v);
    lon_traj.AppendSegment(0.0,
        g_config_param.trajectory_time_length + g_config_param.lattice_epsilon);

    for (double t = 0.0; t < g_config_param.trajectory_time_length;
                               t += g_config_param.trajectory_time_resolution) {
      reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));
    }
  }// 2. other situation exist stop point
  else {
    //2.1 stop point from behind the vehicle center point
    double dist_s = planning_target.stop_point.s - init_s_[0];
    if (dist_s < g_config_param.lattice_epsilon) // dist_s < 0
    {
      PiecewiseAccelerationTrajectory1d lon_traj(init_s_[0], 0.0);
      lon_traj.AppendSegment(0.0,
          g_config_param.trajectory_time_length + g_config_param.lattice_epsilon);

      for (double t = 0.0; t < g_config_param.trajectory_time_length;
           t += g_config_param.trajectory_time_resolution) {
        reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));//all velocity = 0;
      }
      return reference_s_dot;
    }

    double a_comfort = g_config_param.longitudinal_acceleration_upper_bound *  // a = 4.0
                       g_config_param.comfort_acceleration_factor;             // = 0.5
    double d_comfort = -g_config_param.longitudinal_acceleration_lower_bound * // a = -4.5
                       g_config_param.comfort_acceleration_factor;             //= 0.5

    std::shared_ptr<Trajectory1d> lon_ref_trajectory =
        PiecewiseBrakingTrajectoryGenerator::Generate(
            planning_target.stop_point.s, init_s_[0],
            planning_target.cruise_speed,
            init_s_[1], a_comfort, d_comfort,
            g_config_param.trajectory_time_length + g_config_param.lattice_epsilon);

    for (double t = 0.0; t < g_config_param.trajectory_time_length;
         t += g_config_param.trajectory_time_resolution) {
      reference_s_dot.emplace_back(lon_ref_trajectory->Evaluate(1, t));
    }
  }

  return reference_s_dot;
}

bool TrajectoryEvaluator::InterpolateDenseStPoints(
    const std::vector<SpeedPoint>& st_points, double t, double* traj_s) const {
  //CHECK_GT(st_points.size(), 1);
  if (t < st_points[0].t || t > st_points[st_points.size() - 1].t) {
    cout << "AutoTuning InterpolateDenseStPoints Error"<<endl;
    return false;
  }
  for (uint i = 1; i < st_points.size(); ++i) {
    if (t <= st_points[i].t) {
      *traj_s = st_points[i].t;
      return true;
    }
  }
  return false;
}

bool TrajectoryEvaluator::has_more_trajectory_pairs() const {
  if (!g_config_param.enable_auto_tuning) {
    return !cost_queue_.empty();
  } else {
    return !cost_queue_with_components_.empty();
  }
}

std::size_t TrajectoryEvaluator::num_of_trajectory_pairs() const {
  if (!g_config_param.enable_auto_tuning) {
    return cost_queue_.size();
  } else {
    return cost_queue_with_components_.size();
  }
}

std::pair<PtrTrajectory1d, PtrTrajectory1d>
TrajectoryEvaluator::next_top_trajectory_pair() {
  //CHECK(has_more_trajectory_pairs() == true);
  if (!g_config_param.enable_auto_tuning) {
    auto top = cost_queue_.top();
    cost_queue_.pop();
    return top.first;
  } else {
    auto top = cost_queue_with_components_.top();
    cost_queue_with_components_.pop();
    return top.first;
  }
}

double TrajectoryEvaluator::top_trajectory_pair_cost() const {
  if (!g_config_param.enable_auto_tuning) {
    return cost_queue_.top().second;
  } else {
    return cost_queue_with_components_.top().second.second;
  }
}

std::vector<double> TrajectoryEvaluator::top_trajectory_pair_component_cost()
    const {

  if(g_config_param.enable_auto_tuning){
    return cost_queue_with_components_.top().second.first;
  }else{
    cout<<"Error don't enable_auto_tuning but using the "
          "top_trajectory_pair_component_cost() function !!!"<<endl;
    std::vector<double> a;
    return a;
  }

}

}  // namespace planning

