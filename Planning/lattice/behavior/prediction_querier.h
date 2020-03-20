
/**
 * @file
 **/

#ifndef PLANNING_LATTICE_BEHAVIOR_PREDICTION_QUERIER_H_
#define PLANNING_LATTICE_BEHAVIOR_PREDICTION_QUERIER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../../common/obstacle.h"

namespace planning {



class PredictionQuerier {
 public:
  explicit PredictionQuerier(
      const std::vector<const Obstacle*>& obstacles,
      const std::shared_ptr<std::vector<PathPoint>>& ptr_reference_line);

  virtual ~PredictionQuerier() = default;

  std::vector<const Obstacle*> GetObstacles() const;

  double ProjectVelocityAlongReferenceLine(
      const int32_t& obstacle_id,
      const double s,
      const double t) const;

 private:
  std::unordered_map<int32_t, const Obstacle*> id_obstacle_map_;


  std::vector<const Obstacle*> obstacles_;

  std::shared_ptr<std::vector<PathPoint>> ptr_reference_line_;
};


} //end namespace planning


#endif /* MODULES_PLANNING_LATTICE_BEHAVIOR_PREDICTION_QUERIER_H_ */
