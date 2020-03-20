
/**
 * @file
 **/

#ifndef PLANNING_LATTICE_PIECEWISE_BRAKING_TRAJECTORY_GENERATOR_H_
#define PLANNING_LATTICE_PIECEWISE_BRAKING_TRAJECTORY_GENERATOR_H_

#include <memory>

#include "../../lattice/trajectory1d/piecewise_acceleration_trajectory1d.h"


namespace planning {

class PiecewiseBrakingTrajectoryGenerator {
 public:
  PiecewiseBrakingTrajectoryGenerator() = delete;

  static std::shared_ptr<Curve1d> Generate(
      const double s_target, const double s_curr,
      const double v_target, const double v_curr,
      const double a_comfort, const double d_comfort,
      const double max_time);

  static double ComputeStopDistance(const double v,
      const double dec);

  static double ComputeStopDeceleration(const double dist,
      const double v);
};

}  // namespace planning


#endif /* MODULES_PLANNING_LATTICE_PIECEWISE_BRAKING_TRAJECTORY_GENERATOR_H_ */
