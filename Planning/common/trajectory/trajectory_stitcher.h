
/**
 * @file
 **/

#ifndef PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
#define PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_

#include <utility>
#include <vector>
#include "../../common/struct.h"
#include "../../reference_line/reference_line.h"
#include "../../common/trajectory/publishable_trajectory.h"



namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static void TransformLastPublishedTrajectory(const double x_diff,
      const double y_diff, const double theta_diff,
      PublishableTrajectory* prev_trajectory);

  static std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
      const VehicleState& vehicle_state,
      const SLPoint &adc_sl,
      const uint64_t current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory);

 private:
  static std::pair<double, double> ComputePositionProjection(const double x,
      const double y, const TrajectoryPoint& matched_trajectory_point);

  static std::vector<TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const VehicleState& vehicle_state);
};

}  // namespace planning

#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
