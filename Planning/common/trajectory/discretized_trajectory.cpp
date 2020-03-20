
/**
 * @file discretized_trajectory.cc
 **/

#include <algorithm>
#include <limits>
#include <utility>
#include <iostream>

#include "../../math/linear_interpolation.h"
#include "../../common/trajectory/discretized_trajectory.h"


namespace planning {


DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  //CHECK(!trajectory_points.empty())
  //<< "trajectory_points should NOT be empty()";
  trajectory_points_ = trajectory_points;
  int i = 0;
  for(auto & point : trajectory_points_){
      point.relative_time = i * 0.1;
      i++;
  }
}

DiscretizedTrajectory::DiscretizedTrajectory(const ADCTrajectory& trajectory) {
  trajectory_points_.assign(trajectory.trajectory_point.begin(),
                            trajectory.trajectory_point.end());
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time < relative_time;
  };

  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, comp);

  if (it_lower == trajectory_points_.begin()) {
    return trajectory_points_.front();
  } else if (it_lower == trajectory_points_.end()) {
    std::cout<< "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large"<<std::endl;
    return trajectory_points_.back();
  }
  return math::InterpolateUsingLinearApproximation(
      *(it_lower - 1), *it_lower, relative_time);
}

std::uint32_t DiscretizedTrajectory::QueryLowerBoundPoint(
    const double relative_time) const {
  //CHECK(!trajectory_points_.empty());

  if (relative_time >= trajectory_points_.back().relative_time) {
    return trajectory_points_.size() - 1;
  }
  auto func = [](const TrajectoryPoint& tp, const double relative_time) {
    return tp.relative_time < relative_time;
  };
  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, func);
  return std::distance(trajectory_points_.begin(), it_lower);
}

std::uint32_t DiscretizedTrajectory::QueryNearestPoint(
    const Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  std::uint32_t index_min = 0;
  for (std::uint32_t i = 0; i < trajectory_points_.size(); ++i) {
    const Vec2d curr_point(
        trajectory_points_[i].path_point.x,
        trajectory_points_[i].path_point.y);

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr <= dist_sqr_min ) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::
AppendTrajectoryPoint(const TrajectoryPoint& trajectory_point) {
  //if (!trajectory_points_.empty()) {
    //CHECK_GT(trajectory_point.relative_time(),
    //trajectory_points_.back().relative_time());
  //}
  trajectory_points_.push_back(trajectory_point);
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const std::uint32_t index) const {
  //CHECK_LT(index, NumOfPoints());
  return trajectory_points_[index];
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  //CHECK(!trajectory_points_.empty());
  return trajectory_points_.front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  //CHECK(!trajectory_points_.empty());
  return trajectory_points_.back().relative_time -
         trajectory_points_.front().relative_time;
}

double DiscretizedTrajectory::GetSpatialLength() const {
  //CHECK(!trajectory_points_.empty());
  return trajectory_points_.back().path_point.s -
         trajectory_points_.front().path_point.s;
}

}  // namespace planning

