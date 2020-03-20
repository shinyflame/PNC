
/**
 * @file trajectory.h
 **/

#ifndef PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
#define PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
#include "../struct.h"

namespace planning {

class Trajectory {
 public:
  Trajectory() = default;

  virtual ~Trajectory() = default;

  virtual TrajectoryPoint Evaluate(
      const double relative_time) const = 0;

  virtual TrajectoryPoint StartPoint() const = 0;

  virtual double GetTemporalLength() const = 0;

  virtual double GetSpatialLength() const = 0;
};

}  // namespace planning


#endif  // MODULES_PLANNING_COMMON_TRAJECTORY_TRAJECTORY_H_
