
/**
 * @file
 **/

#include "../constraint_checker/constraint_checker1d.h"


extern planning::ConfigParam g_config_param;

namespace planning {


namespace {

inline bool fuzzy_within(const double v, const double lower, const double upper,
                         const double e = 1.0e-4) {
  return v > lower - e && v < upper + e;
}
}  // namespace

bool ConstraintChecker1d::IsValidLongitudinalTrajectory(
    const Curve1d& lon_trajectory) {
  double t = 0.1;

  while (t < lon_trajectory.ParamLength()) {
    double v = lon_trajectory.Evaluate(1, t);  // evalute_v
    if (!fuzzy_within(v, g_config_param.speed_lower_bound,
                         g_config_param.speed_upper_bound)) {
      return false;
    }

    double a = lon_trajectory.Evaluate(2, t);  // evaluat_a
    if (!fuzzy_within(a, g_config_param.longitudinal_acceleration_lower_bound,
                         g_config_param.longitudinal_acceleration_upper_bound)) {
      return false;
    }

    double j = lon_trajectory.Evaluate(3, t); // evaluat_jerk
    if (!fuzzy_within(j, g_config_param.longitudinal_jerk_lower_bound,
                         g_config_param.longitudinal_jerk_upper_bound)) {
      return false;
    }
    t += g_config_param.trajectory_time_resolution;
  }
  return true;
}

bool ConstraintChecker1d::IsValidLateralTrajectory(
    const Curve1d& lat_trajectory, const Curve1d& lon_trajectory,double l0) {
  double t = 0.0;
  double s0 = lon_trajectory.Evaluate(0, 0.0);//modify by sxl 20190524
  while (t < lon_trajectory.ParamLength()) {

    double s  = lon_trajectory.Evaluate(0, t);
    double ds = lon_trajectory.Evaluate(1, t);
    double d2s= lon_trajectory.Evaluate(2, t);
    double theta_s = s - s0;
    double l  = lat_trajectory.Evaluate(0, theta_s);//modify by sxl 20190524
    double dl = lat_trajectory.Evaluate(1, theta_s);//modify by sxl 20190524
    double d2l= lat_trajectory.Evaluate(2, theta_s);//modify by sxl 20190524
    //1, laterl in boundary?
    if(l0 > -g_config_param.lat_offset_bound  &&
       l0 <  g_config_param.lat_offset_bound )
      {
        if(l < -g_config_param.lat_offset_bound - 0.5 ||
           l >  g_config_param.lat_offset_bound + 0.5)
          {
            cout<<"later l = "<<l
                <<" beyond defaul tlat_offset_bound [ "
                <<-g_config_param.lat_offset_bound<<" , "
                << g_config_param.lat_offset_bound<<" ]"<<endl;
            return false;
          }
      }

    double a = 0.0;
    if (s < lat_trajectory.ParamLength() + s0) {
      a = d2l * ds * ds + dl * d2s;
    }else{
      t += g_config_param.trajectory_time_resolution;
      continue;
    }

    if (!fuzzy_within(a, -g_config_param.lateral_acceleration_bound,//4.0 m/^2
                          g_config_param.lateral_acceleration_bound)) {
        cout<<"later a = " <<a<<endl;
        return false;
    }

    // this is not accurate, just an approximation...
#if 0
    double j = 0.0;
    if (s < lat_trajectory.ParamLength() + s0) {//modify by sxl 20190524
      j = lat_trajectory.Evaluate(3, s - s0) * lon_trajectory.Evaluate(3, t);
    }

    if (!fuzzy_within(j, -g_config_param.lateral_jerk_bound, //4.0 m/s^3
                          g_config_param.lateral_jerk_bound)) {
        cout<<"later j = " <<j<<endl;
        return false;
    }
#endif
    t += g_config_param.trajectory_time_resolution;
  }

  return true;
}

}  // namespace planning

