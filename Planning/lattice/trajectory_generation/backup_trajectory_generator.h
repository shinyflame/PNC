
/**
 * @file
 **/

#ifndef PLANNING_LATTICE_TRAJECTORY_GENERATION_BACKUP_TRAJECTORY_H_
#define PLANNING_LATTICE_TRAJECTORY_GENERATION_BACKUP_TRAJECTORY_H_

#include <array>
#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "../../lattice/trajectory_generation/trajectory1d_generator.h"
#include "../../common/trajectory/discretized_trajectory.h"
#include "../../lattice/trajectory1d/constant_deceleration_trajectory1d.h"
#include "../../constraint_checker/collision_checker.h"
#include "../../math/curve1d/curve1d.h"

extern ConfigParam g_config_param;

namespace planning {



class BackupTrajectoryGenerator {
 public:
  typedef std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
      Trajectory1dPair;
  typedef std::pair<Trajectory1dPair, double> PairCost;

  BackupTrajectoryGenerator(
      const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
      const double init_relative_time,
      const std::shared_ptr<CollisionChecker>& ptr_collision_checker,
      const Trajectory1dGenerator* trajectory1d_generator);

  DiscretizedTrajectory GenerateTrajectory(
      const std::vector<PathPoint>& discretized_ref_points);

 private:
  void GenerateTrajectory1dPairs(const std::array<double, 3>& init_s,
                                 const std::array<double, 3>& init_d);

  double init_relative_time_;

  std::shared_ptr<CollisionChecker> ptr_collision_checker_;

  const Trajectory1dGenerator* ptr_trajectory1d_generator_;

  struct CostComparator
      : public std::binary_function<const Trajectory1dPair&,
                                    const Trajectory1dPair&, bool> {
    bool operator()(const Trajectory1dPair& left,
                    const Trajectory1dPair& right) const {
      auto lon_left = left.first;
      auto lon_right = right.first;
      auto s_dot_left = lon_left->Evaluate(1, g_config_param.trajectory_time_length);
      auto s_dot_right = lon_right->Evaluate(1, g_config_param.trajectory_time_length);
      if (s_dot_left < s_dot_right) {
        return true;
      }
      return false;
    }
  };

  std::priority_queue<Trajectory1dPair, std::vector<Trajectory1dPair>,
                      CostComparator>
      trajectory_pair_pqueue_;
};

}  // namespace planning


#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_BACKUP_TRAJECTORY_H_
