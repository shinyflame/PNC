#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "../math/box2d.h"
#include "../math/polygon2d.h"
#include "../math/vec2d.h"
#include "../common/struct.h"
#include "../math/util.h"
#include "../common/indexed_list.h"
#include <list>
#include <memory>
#include <string>
#include <vector>

using namespace math;

namespace planning {


class Obstacle {
 public:
  Obstacle() = default;

  Obstacle(const std::int32_t &id,
           const perception::PerceptionObstacle &perception_obstacle);

  Obstacle(const std::int32_t &id,
           const perception::PerceptionObstacle &perception,
           const prediction::Trajectory &trajectory);

  const std::int32_t &Id() const;
  void SetId(const std::int32_t &id) { id_ = id; }

  std::int32_t PerceptionId() const;

  double Speed() const;

  bool IsStatic() const;
  bool IsVirtual() const;

  TrajectoryPoint GetPointAtTime(const double time) const;

  Box2d GetBoundingBox(
      const TrajectoryPoint &point) const;
  /**
   * @brief get the perception bounding box
   */
  const Box2d &PerceptionBoundingBox() const;

  /**
   * @brief get the perception polygon for the obstacle. It is more precise than
   * bounding box
   */
  const Polygon2d &PerceptionPolygon() const;

  const prediction::Trajectory &Trajectory() const;

  void AddTrajectoryPoint(TrajectoryPoint trajectoryPoint);
  bool HasTrajectory() const;

  const perception::PerceptionObstacle &Perception() const;

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const prediction::PredictionObstacles &predictions);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::int32_t &id, const Box2d &obstacle_box);

  static bool IsStaticObstacle(
      const perception::PerceptionObstacle &perception_obstacle);

  static bool IsVirtualObstacle(
      const perception::PerceptionObstacle &perception_obstacle);

  static bool IsValidTrajectoryPoint(const TrajectoryPoint &point);

 private:
  std::int32_t id_;
  std::int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;
  prediction::Trajectory trajectory_;//prediction::Trajectory trajectory_;
  perception::PerceptionObstacle perception_obstacle_;//perception::PerceptionObstacle perception_obstacle_;
  Box2d perception_bounding_box_;
  Polygon2d perception_polygon_;
};

typedef IndexedList<int32_t, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<int32_t, Obstacle> ThreadSafeIndexedObstacles;
void Obstacle_IsStaticObstacle();
void ObstacleTest_CreateObstacles();


} //end namespace planning


#endif // OBSTACLE_H
