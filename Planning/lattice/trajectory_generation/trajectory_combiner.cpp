

#include "../../lattice/trajectory_generation/trajectory_combiner.h"
#include <algorithm>
#include "../../math/cartesian_frenet_conversion.h"
#include "../../math/path_matcher.h"

extern ConfigParam g_config_param;
namespace planning {


using math::CartesianFrenetConverter;
using math::PathMatcher;


DiscretizedTrajectory TrajectoryCombiner::
  Combine(   const std::vector<PathPoint>& reference_line,
             const Curve1d& lon_trajectory,
             const Curve1d& lat_trajectory,
             const double init_relative_time,
             const bool   has_stop_point,
             Frame *frame,
             ReferenceLineInfo* reference_line_info)
{
  cout<<"init_relative_time = "<<init_relative_time<<endl;
  DiscretizedTrajectory combined_trajectory;
  double s0 = lon_trajectory.Evaluate(0, 0.0);
  double s_ref_max = reference_line.back().s;
  double accumulated_trajectory_s = 0.0;
  PathPoint prev_trajectory_point;

  double last_s = -g_config_param.lattice_epsilon;
  double t_param = 0.0;
  //int i = 0;
  while (t_param < g_config_param.trajectory_time_length) {
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about t_param > lon_trajectory.ParamLength() situation
    double s = lon_trajectory.Evaluate(0, t_param);
    if (last_s > 0.0) {
      s = std::max(last_s, s);
    }
    if(t_param < 0.8 && !has_stop_point)
      if(abs(s-last_s) < 0.01){
         t_param = t_param + g_config_param.trajectory_time_resolution;//0.1s
         continue;
      }
    last_s = s;

    double s_dot =
        std::max(g_config_param.lattice_epsilon, lon_trajectory.Evaluate(1, t_param));
    double s_ddot = lon_trajectory.Evaluate(2, t_param);

    if (s > s_ref_max) {
      break;
    }

    double s_param = s - s0;
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about s_param > lat_trajectory.ParamLength() situation
    double d = lat_trajectory.Evaluate(0, s_param);
    double d_prime = lat_trajectory.Evaluate(1, s_param);
    double d_pprime  = lat_trajectory.Evaluate(2, s_param);

    int self_index = s /  g_config_param.discretion_length;
    int start_index = self_index -10;
    if(start_index < 0) start_index =0;
    int end_index = self_index +10;
    if(end_index > reference_line.size() -1) end_index = reference_line.size() - 1;
    //PathPoint matched_ref_point = PathMatcher::MatchToPath(reference_line, s,start_index,end_index);
    auto & line_provider = frame->reference_line_provider();
    PathPoint matched_ref_point =
    line_provider.GetMatchPointFromS(s,
    reference_line_info->reference_line_mutable()->GetReferenceLineWithParam());
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s;
    const double rx = matched_ref_point.x;
    const double ry = matched_ref_point.y;
    const double rtheta = matched_ref_point.theta;
    const double rkappa = matched_ref_point.kappa;
    const double rdkappa = matched_ref_point.dkappa;

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};

    CartesianFrenetConverter::frenet_to_cartesian(
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
        &theta, &kappa, &v, &a);

    if(t_param > 0.0){
      double delta_x = x - prev_trajectory_point.x;
      double delta_y = y - prev_trajectory_point.y;
      double delta_s = std::hypot(delta_x, delta_y);
      accumulated_trajectory_s += delta_s;
    }

    //theta ( -M_PI ~ M_PI )
    theta -= M_PI/2.0;//heading convert yaw
    if(theta > M_PI){
        theta -= 2.0*M_PI;
     } else if(theta < -M_PI){
        theta += 2.0*M_PI;
     }
    TrajectoryPoint trajectory_point;
    trajectory_point.path_point.x=x;
    trajectory_point.path_point.y=y;
    trajectory_point.path_point.s= accumulated_trajectory_s;
    trajectory_point.path_point.theta= theta ;
    trajectory_point.path_point.kappa= kappa;
    trajectory_point.v=v;
    trajectory_point.a=a;
    trajectory_point.relative_time=t_param + init_relative_time;

    combined_trajectory.AppendTrajectoryPoint(trajectory_point);

    t_param = t_param + g_config_param.trajectory_time_resolution;//0.1s

    prev_trajectory_point = trajectory_point.path_point;
  }
  return combined_trajectory;
}

DiscretizedTrajectory TrajectoryCombiner::
  Combine(   const std::vector<PathPoint>& reference_line,
             const Curve1d& lon_trajectory,
             const Curve1d& lat_trajectory,
             const double init_relative_time,
             const bool has_stop_point)
{
  //cout<<"init_relative_time = "<<init_relative_time<<endl;
  DiscretizedTrajectory combined_trajectory;
  double s0 = lon_trajectory.Evaluate(0, 0.0);
  double s_ref_max = reference_line.back().s;
  double accumulated_trajectory_s = 0.0;
  PathPoint prev_trajectory_point;

  double last_s = -g_config_param.lattice_epsilon;
  double t_param = 0.0;
  //int i = 0;
  while (t_param < g_config_param.trajectory_time_length) {
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about t_param > lon_trajectory.ParamLength() situation
    double s = lon_trajectory.Evaluate(0, t_param);
    if (last_s > 0.0) {
      s = std::max(last_s, s);
    }
    if(t_param < 0.8 && !has_stop_point)
      if(abs(s-last_s) < 0.01){
         t_param = t_param + g_config_param.trajectory_time_resolution;//0.1s
         continue;
      }
    last_s = s;

    double s_dot =
        std::max(g_config_param.lattice_epsilon, lon_trajectory.Evaluate(1, t_param));
    double s_ddot = lon_trajectory.Evaluate(2, t_param);

    if (s > s_ref_max) {
      break;
    }

    double s_param = s - s0;
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about s_param > lat_trajectory.ParamLength() situation
    double d = lat_trajectory.Evaluate(0, s_param);
    double d_prime = lat_trajectory.Evaluate(1, s_param);
    double d_pprime  = lat_trajectory.Evaluate(2, s_param);

    int self_index = s /  g_config_param.discretion_length;
    int start_index = self_index - 10;
    if(start_index < 0) start_index =0;
    int end_index = self_index + 10;
    if(end_index > reference_line.size() -1) end_index = reference_line.size() - 1;
    PathPoint matched_ref_point = PathMatcher::MatchToPath(reference_line, s,start_index,end_index);

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s;
    const double rx = matched_ref_point.x;
    const double ry = matched_ref_point.y;
    const double rtheta = matched_ref_point.theta;
    const double rkappa = matched_ref_point.kappa;
    const double rdkappa = matched_ref_point.dkappa;

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};

    CartesianFrenetConverter::frenet_to_cartesian(
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
        &theta, &kappa, &v, &a);

    if(t_param > 0.0){
      double delta_x = x - prev_trajectory_point.x;
      double delta_y = y - prev_trajectory_point.y;
      double delta_s = std::hypot(delta_x, delta_y);
      accumulated_trajectory_s += delta_s;
    }

    //theta ( -M_PI ~ M_PI )
    theta -= M_PI/2.0;//heading convert yaw
    if(theta > M_PI){
        theta -= 2.0*M_PI;
     } else if(theta < -M_PI){
        theta += 2.0*M_PI;
     }
    TrajectoryPoint trajectory_point;
    trajectory_point.path_point.x=x;
    trajectory_point.path_point.y=y;
    trajectory_point.path_point.s= accumulated_trajectory_s;
    trajectory_point.path_point.theta= theta ;
    trajectory_point.path_point.kappa= kappa;
    trajectory_point.v=v;
    trajectory_point.a=a;
    trajectory_point.relative_time=t_param + init_relative_time;

    combined_trajectory.AppendTrajectoryPoint(trajectory_point);

    t_param = t_param + g_config_param.trajectory_time_resolution;//0.1s

    prev_trajectory_point = trajectory_point.path_point;
  }
  return combined_trajectory;
}


}  // namespace planning

