
/**
 * @file ego_info.h
 **/

#ifndef MODULES_PLANNING_COMMON_EGO_INFO_H_
#define MODULES_PLANNING_COMMON_EGO_INFO_H_

#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>
#include "macro.h"
#include "obstacle.h"
#include "../reference_line/reference_line.h"


namespace planning {

class EgoInfo {
 public:
  ~EgoInfo() = default;

  bool Update(const TrajectoryPoint& start_point,
              const VehicleState& vehicle_state,
              const std::vector<const Obstacle*>& obstacles);
  void Clear();

  TrajectoryPoint start_point() const { return start_point_; }

  VehicleState vehicle_state() const { return vehicle_state_; }

  double front_clear_distance() const { return front_clear_distance_; }

  friend void EgoInfoTestEgoInfoSimpleTest();

 private:
  //FRIEND_TEST(EgoInfoTest, EgoInfoSimpleTest);

  void set_vehicle_state(const VehicleState& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void set_start_point(const TrajectoryPoint& start_point) {
    start_point_ = start_point;
  }

  void CalculateFrontObstacleClearDistance(
      const std::vector<const Obstacle*>& obstacles);

  // stitched point (at stitching mode)
  // or real vehicle point (at non-stitching mode)
  TrajectoryPoint start_point_;

  // ego vehicle state
  VehicleState vehicle_state_;

  double front_clear_distance_ = std::numeric_limits<double>::max();

  VehicleParam ego_vehicle_config_;

  DECLARE_SINGLETON(EgoInfo);
};
void EgoInfoTestEgoInfoSimpleTest();
}  // namespace planning

#endif  // MODULES_PLANNING_COMMON_EGO_INFO_H_
