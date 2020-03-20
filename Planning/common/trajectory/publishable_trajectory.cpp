
/**
 * @file publishable_trajectory.cpp
 **/

#include "../../common/trajectory/publishable_trajectory.h"
#include <utility>

#if 1
namespace planning {

PublishableTrajectory::PublishableTrajectory(const double header_time,
                           const DiscretizedTrajectory& discretized_trajectory)
    : DiscretizedTrajectory(discretized_trajectory),header_time_(header_time) {}


PublishableTrajectory::PublishableTrajectory(const ADCTrajectory& trajectory_pb)
    : DiscretizedTrajectory(trajectory_pb),
      header_time_(trajectory_pb.header.timestamp_ms) {}



double PublishableTrajectory::header_time() const { return header_time_; }

void PublishableTrajectory::PopulateTrajectoryProtobuf(
                                   ADCTrajectory* trajectory_pb) const {
  //CHECK_NOTNULL(trajectory_pb);
  trajectory_pb->header.timestamp_ms = header_time_;
  trajectory_pb->trajectory_point = trajectory_points_;

  if (!trajectory_points_.empty()) {
    const auto& last_tp = trajectory_points_.back();
    trajectory_pb->total_path_length = (last_tp.path_point.s);
    trajectory_pb->total_path_time   = (last_tp.relative_time);
  }
}

void PublishableTrajectory::PopulateTrajectoryProtobuf(
                                           PbTrajectory* trajectory_pb) const {
  //CHECK_NOTNULL(trajectory_pb);
  trajectory_pb->pb_header.plan_timestamp = header_time_;
  //trajectory_pb->trajectory_points.clear();
  trajectory_pb->trajectory_points = trajectory_points_;
}

}  // namespace planning

#endif
