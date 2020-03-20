
/**
 * @file publishable_trajectory.h
 **/

#ifndef PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
#define PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_

#include <vector>
#include "./discretized_trajectory.h"


namespace planning {

class PublishableTrajectory : public DiscretizedTrajectory {
 public:
  PublishableTrajectory() = default;

  PublishableTrajectory(const double header_time,
                        const DiscretizedTrajectory& discretized_trajectory);
  /**
   * Create a publishable trajectory based on a trajectory protobuf
   */
  explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

  double header_time() const;

  void PopulateTrajectoryProtobuf(ADCTrajectory* trajectory_pb) const;
  void PopulateTrajectoryProtobuf(PbTrajectory * trajectory_pb) const;
 private:
  //double header_time_ = 0.0;
  uint64_t header_time_ = 0;
};

void basic_test_PublishableTrajectory() ;
}  // namespace planning


#endif  // PLANNING_COMMON_TRAJECTORY_PUBLISHABLE_TRAJECTORY_H_
