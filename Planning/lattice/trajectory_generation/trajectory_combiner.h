
/**
 * @file
 **/

#ifndef PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY_COMBINER_H_
#define PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY_COMBINER_H_

#include <vector>
#include "../../common/struct.h"
#include "../../common/trajectory/discretized_trajectory.h"
#include "../../math/curve1d/curve1d.h"
#include "../../reference_line/reference_line_provider.h"
#include "../../common/frame.h"
#include "../../common/reference_line_info.h"
namespace planning {

class TrajectoryCombiner {
 public:
  static DiscretizedTrajectory Combine(
      const std::vector<PathPoint>& reference_line,
      const Curve1d& lon_trajectory, const Curve1d& lat_trajectory,
      const double init_relative_time,const bool has_stop_point,
      Frame *frame,
      ReferenceLineInfo* reference_line_info);

  static DiscretizedTrajectory Combine(
      const std::vector<PathPoint>& reference_line,
      const Curve1d& lon_trajectory, const Curve1d& lat_trajectory,
      const double init_relative_time,const bool has_stop_point);
};

}  // namespace planning


#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY_COMBINER_H_
