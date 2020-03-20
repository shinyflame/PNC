
/**
 * @file
 **/

#ifndef PLANNING_CONSTRAINT_CHECKER_CONSTRAINT_CHECKER1D_H_
#define PLANNING_CONSTRAINT_CHECKER_CONSTRAINT_CHECKER1D_H_

#include <vector>

#include "../common/trajectory/discretized_trajectory.h"
#include "../math/curve1d/curve1d.h"


namespace planning {

class ConstraintChecker1d {
 public:
  ConstraintChecker1d() = delete;

  static bool IsValidLongitudinalTrajectory(const Curve1d& lon_trajectory);

  static bool IsValidLateralTrajectory(const Curve1d& lat_trajectory,
                                       const Curve1d& lon_trajectory,double l0);
};

}  // namespace planning


#endif  // MODULES_PLANNING_CONSTRAINT_CHECKER_CONSTRAINT_CHECKER1D_H_
