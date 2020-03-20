
/**
 * @file
 **/

#include "../../lattice/trajectory_generation/piecewise_braking_trajectory_generator.h"
#include <cmath>



namespace planning {

std::shared_ptr<Curve1d> PiecewiseBrakingTrajectoryGenerator::Generate(
    const double s_target, const double s_curr, const double v_target,
    const double v_curr, const double a_comfort, const double d_comfort,
    const double max_time) {
  std::shared_ptr<PiecewiseAccelerationTrajectory1d> ptr_trajectory =
               std::make_shared<PiecewiseAccelerationTrajectory1d>(s_curr, v_curr);

  double s_dist = s_target - s_curr;//计算停车距离

  double comfort_stop_dist = ComputeStopDistance(v_curr, d_comfort);//计算以最佳减速度的方式停车距离

  // if cannot stop using comfort deceleration, then brake in the beginning.
  if (comfort_stop_dist > s_dist) {//如果最佳减速度减速距离不能停车
    double stop_d = ComputeStopDeceleration(s_dist, v_curr);//计算加速度
    double stop_t = v_curr / stop_d;//计算停车时间
    ptr_trajectory->AppendSegment(-stop_d, stop_t);

    if (ptr_trajectory->ParamLength() < max_time) {//如果不够8s钟，则后面的轨迹点补全为加速度为0
      ptr_trajectory->AppendSegment(0.0,
                                    max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;
  }

  // otherwise, the vehicle can stop from current speed with comfort brake.
  if (v_curr > v_target) {
    double t_cruise = (s_dist - comfort_stop_dist) / v_target;//求巡航时间
    double t_rampdown = (v_curr - v_target) / d_comfort;//求减速到巡航速度时间
    double t_dec = v_target / d_comfort;//求减速时间

    ptr_trajectory->AppendSegment(-d_comfort, t_rampdown);//减速到巡航速度
    ptr_trajectory->AppendSegment(0.0, t_cruise);//巡航
    ptr_trajectory->AppendSegment(-d_comfort, t_dec);//减速到0

    if (ptr_trajectory->ParamLength() < max_time) {
      ptr_trajectory->AppendSegment(0.0,
                                    max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;

  } else {
    double t_rampup = (v_target - v_curr) / a_comfort;//求加速到巡航速度时间
    double t_rampdown = (v_target - v_curr) / d_comfort;//以减速度加速到巡航速度的时间
    double s_ramp = (v_curr + v_target) * (t_rampup + t_rampdown) * 0.5;//加速行驶的路程

    double s_rest = s_dist - s_ramp - comfort_stop_dist;//求实际停车距离与理论计算停车距离之差
    if (s_rest > 0) {//如果大于零
      double t_cruise = s_rest / v_target;//求巡航的时间
      double t_dec = v_target / d_comfort;//求减速到零的时间

      // construct the trajectory
      ptr_trajectory->AppendSegment(a_comfort, t_rampup);//加速
      ptr_trajectory->AppendSegment(0.0, t_cruise);//巡航
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);//减速

      if (ptr_trajectory->ParamLength() < max_time) {
        ptr_trajectory->AppendSegment(0.0,
                                      max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    } else {//xiao于等于零
      double s_rampup_rampdown = s_dist - comfort_stop_dist;//加速减速时间
      double v_max = std::sqrt(v_curr * v_curr +
                               2.0 * a_comfort * d_comfort * s_rampup_rampdown /
                                   (a_comfort + d_comfort));

      double t_acc = (v_max - v_curr) / a_comfort;//加速时间
      double t_dec = v_max / d_comfort;//减速时间

      // construct the trajectory
      ptr_trajectory->AppendSegment(a_comfort, t_acc);//加速
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);//减速

      if (ptr_trajectory->ParamLength() < max_time) {
        ptr_trajectory->AppendSegment(0.0,
                                      max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    }
  }
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDistance(
    const double v, const double dec) {
  //CHECK(dec > 0.0);
  return v * v / dec * 0.5;
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDeceleration(
    const double dist, const double v) {
  return v * v / dist * 0.5;
}

}  // namespace planning

