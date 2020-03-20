
/**
 * @file
 **/

#ifndef PLANNING_COMMON_LAG_PREDICTION_H_
#define PLANNING_COMMON_LAG_PREDICTION_H_

#include <unordered_map>
#include <unordered_set>
#include "../common/struct.h"
#include "./get_data.h"

namespace planning {

class LagPrediction {
 public:
  LagPrediction(uint32_t min_appear_num, uint32_t max_disappear_num);
  void GetLaggedPrediction(prediction::PredictionObstacles* obstacles) const;

  struct LagInfo {
    uint32_t last_observed_seq = 0;
    double last_observed_time = 0.0;
    uint32_t count = 0;
    const prediction::PredictionObstacle* obstacle_ptr = nullptr;
  };

 private:
  void AddObstacleToPrediction(
      double delay_sec, const prediction::PredictionObstacle& obstacle,
      prediction::PredictionObstacles* obstacles) const;

  uint32_t min_appear_num_ = 0;
  uint32_t max_disappear_num_ = 0;
};

}  // namespace planning


#endif  // namespace PLANNING_COMMON_LAG_PREDICTION_H_
