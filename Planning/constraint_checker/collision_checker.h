
/**
 * @file
 **/

#ifndef PLANNING_CONSTRAINT_CHECKER_COLLISION_CHECKER_H_
#define PLANNING_CONSTRAINT_CHECKER_COLLISION_CHECKER_H_

#include <array>
#include <memory>
#include <vector>

#include "../math/box2d.h"
#include "../math/polygon2d.h"
#include "../common/obstacle.h"
#include "../common/reference_line_info.h"
#include "../common/trajectory/discretized_trajectory.h"
#include "../lattice/behavior/path_time_graph.h"


namespace planning {

class CollisionChecker {
 public:
  explicit CollisionChecker(
      const std::vector<const Obstacle*>& obstacles,
      const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<PathPoint>& discretized_reference_line,
      const ReferenceLineInfo* ptr_reference_line_info,
      const std::shared_ptr<PathTimeGraph>& ptr_path_time_graph,
      const RouteType route_type);

  bool InCollision(const DiscretizedTrajectory& discretized_trajectory);

 private:
  void BuildPredictedEnvironment(
      const std::vector<const Obstacle*>& obstacles,
      const double ego_vehicle_s,
      const double ego_vehicle_d,
      const std::vector<PathPoint>& discretized_reference_line);

  bool IsEgoVehicleInLane(const double ego_vehicle_s,
                          const double ego_vehicle_d);

  bool IsObstacleBehindEgoVehicle(
      const Obstacle* obstacle, const double ego_vehicle_s,
      const std::vector<PathPoint>& discretized_reference_line);
  void ComputeSafeEdge(RouteType route_type);

 private:
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::vector<std::vector<Box2d>> predicted_bounding_rectangles_;
   ///add polygons @20190902
  std::vector<std::vector<math::Polygon2d>> predicted_polygons_;
  bool is_get_stop_obs_;
  double stop_s_;
  double slide_clean_right_min_;
  double slide_clean_left_max_ ;
  double non_slide_right_min_ ;
  double non_slide_left_max_ ;
};

}  // namespace planning

#endif  // MODULES_PLANNING_CONSTRAINT_CHECKER_COLLISION_CHECKER_H_
