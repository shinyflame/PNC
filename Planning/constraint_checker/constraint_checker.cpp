
/**
 * @file constraint_checker.cc
 **/

#include "../constraint_checker/constraint_checker.h"


extern planning::ConfigParam g_config_param;

namespace planning {



namespace {
template <typename T>
bool WithinRange(const T v, const T lower, const T upper) {
  return lower <= v && v <= upper;
}
}  // namespace

ConstraintChecker::Result ConstraintChecker::ValidTrajectory(
                                           const DiscretizedTrajectory& trajectory)
{
  const double kMaxCheckRelativeTime = g_config_param.trajectory_time_length;
  for ( const auto & p : trajectory.trajectory_points()) {

//    double t = p.relative_time;
//    if (t > kMaxCheckRelativeTime) {//8.0s
//      break;
//    }
    double lon_v = p.v;
    if (!WithinRange(lon_v, g_config_param.speed_lower_bound,
                            g_config_param.speed_upper_bound)) {//-0.1
//      cout << "Velocity at relative time " << t                 //11.2
//           << " exceeds bound, value: " << lon_v << ", bound ["
//           << g_config_param.speed_lower_bound << ", "
//           << g_config_param.speed_upper_bound
//           << "]."<<endl;
      return Result::LON_VELOCITY_OUT_OF_BOUND;
    }

    double lon_a = p.a;
    if (!WithinRange(lon_a, g_config_param.longitudinal_acceleration_lower_bound,//-4.5
                     g_config_param.longitudinal_acceleration_upper_bound)) {//4.0
//      cout << "Longitudinal acceleration at relative time " << t
//           << " exceeds bound, value: " << lon_a << ", bound ["
//           << g_config_param.longitudinal_acceleration_lower_bound << ", "
//           << g_config_param.longitudinal_acceleration_upper_bound << "]."<<endl;
      return Result::LON_ACCELERATION_OUT_OF_BOUND;
    }

    double kappa = p.path_point.kappa;

    if (!WithinRange(kappa, -g_config_param.kappa_bound,
                             g_config_param.kappa_bound)) {
//      cout<< "Kappa at relative time " << t
//          << " exceeds bound, value: " << kappa << ", bound ["
//          << -g_config_param.kappa_bound<<", "<< g_config_param.kappa_bound<<"]."
//          <<endl;
      return Result::CURVATURE_OUT_OF_BOUND;
    }

  }
  return Result::VALID;

  for (std::size_t i = 1; i < trajectory.NumOfPoints(); ++i) {
    const auto& p0 = trajectory.TrajectoryPointAt(i - 1);
    const auto& p1 = trajectory.TrajectoryPointAt(i);
//    double s = p1.path_point.s;
//    if(s > g_config_param.speed_upper_bound * 0.2 *
//            g_config_param.prediction_point_num) {
//        break;
//    }
//    if (p1.relative_time > kMaxCheckRelativeTime) {//8.0s
//      break;
//    }

    double t = p0.relative_time;

    double dt = p1.relative_time - p0.relative_time;
    double d_lon_a = p1.a - p0.a;
    double lon_jerk = d_lon_a / dt;
    if (!WithinRange(lon_jerk, g_config_param.longitudinal_jerk_lower_bound,// 4.0
                               g_config_param.longitudinal_jerk_upper_bound)) {
     cout << "Longitudinal jerk at relative time " << t
          << " exceeds bound, value: " << lon_jerk << ", bound ["
          << g_config_param.longitudinal_jerk_lower_bound << ", "
          << g_config_param.longitudinal_jerk_upper_bound << "]."<<endl;
      return Result::LON_JERK_OUT_OF_BOUND;
    }

    double lat_a = p1.v * p1.v * p1.path_point.kappa;
    if (!WithinRange(lat_a, -g_config_param.lateral_acceleration_bound,//4.0
                     g_config_param.lateral_acceleration_bound)) {
      cout<< "Lateral acceleration at relative time " << t
          << " exceeds bound, value: " << lat_a << ", bound ["
          << -g_config_param.lateral_acceleration_bound << ", "
          << g_config_param.lateral_acceleration_bound << "]."<<endl;
      return Result::LAT_ACCELERATION_OUT_OF_BOUND;
    }

    // TODO(zhangyajia): this is temporarily disabled
    // due to low quality reference line.
    /**
    double d_lat_a = p1.v() * p1.v() * p1.path_point().kappa() -
                     p0.v() * p0.v() * p0.path_point().kappa();
    double lat_jerk = d_lat_a / dt;
    if (!WithinRange(lat_jerk, -g_config_param.lateral_jerk_bound,
                     g_config_param.lateral_jerk_bound)) {
      ADEBUG << "Lateral jerk at relative time " << t
             << " exceeds bound, value: " << lat_jerk << ", bound ["
             << -g_config_param.lateral_jerk_bound << ", " << g_config_param.lateral_jerk_bound
             << "].";
      return Result::LAT_JERK_OUT_OF_BOUND;
    }
    **/
  }

  return Result::VALID;
}

}  // namespace planning

