
/**
 * @file
 **/

#ifndef PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
#define PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_

#include <vector>
#include "../struct.h"
#include "../../../Planning/math/vec2d.h"
#include "../trajectory/trajectory.h"

using namespace math;

namespace planning {

class DiscretizedTrajectory : public Trajectory {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  TrajectoryPoint StartPoint() const override;

  double GetTemporalLength() const override;

  double GetSpatialLength() const override;

  TrajectoryPoint Evaluate(const double relative_time) const override;

  virtual uint32_t QueryLowerBoundPoint(const double relative_time) const;

  virtual uint32_t QueryNearestPoint(const Vec2d& position) const;

  virtual void AppendTrajectoryPoint(
      const TrajectoryPoint& trajectory_point);

  template <typename Iter>
  void PrependTrajectoryPoints(Iter begin, Iter end) {
    if (!trajectory_points_.empty() && begin != end) {
        trajectory_points_.erase(trajectory_points_.begin());
        trajectory_points_.insert(trajectory_points_.begin(), begin, end);
    }

  }

  const TrajectoryPoint& TrajectoryPointAt(
      const std::uint32_t index) const;

  uint32_t NumOfPoints() const;

  const std::vector<TrajectoryPoint>& trajectory_points() const;
  std::vector<TrajectoryPoint>& trajectory_points();

  virtual void Clear();

 protected:
  std::vector<TrajectoryPoint> trajectory_points_;
};

inline std::uint32_t DiscretizedTrajectory::NumOfPoints() const {
  return trajectory_points_.size();
}

inline const std::vector<TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() const {
  return trajectory_points_;
}

inline std::vector<TrajectoryPoint>&
DiscretizedTrajectory::trajectory_points() {
  return trajectory_points_;
}

inline void DiscretizedTrajectory::SetTrajectoryPoints(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  trajectory_points_ = trajectory_points;
}

inline void DiscretizedTrajectory::Clear() { trajectory_points_.clear(); }

void basic_test_DiscretizedTrajectory();
}  // namespace planning


#endif  // PLANNING_COMMON_TRAJECTORY_DISCRETIZED_TRAJECTORY_H_
