

#ifndef PATH_TIME_GRAPH_H
#define PATH_TIME_GRAPH_H


#include "../../math/linear_interpolation.h"
#include "../../common/reference_line_info.h"
#include "../../common/struct.h"
#include "../../common/obstacle.h"
#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace planning {



class PathTimeGraph {
 public:
  PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                const std::vector<PathPoint>& discretized_ref_points,
                const ReferenceLineInfo* ptr_reference_line_info,
                const double s_start, const double s_end,
                const double t_start, const double t_end,
                const std::array<double, 3>& init_d );

  const std::vector<PathTimeObstacle>& GetPathTimeObstacles() const;

  bool GetPathTimeObstacle(const int32_t&      obstacle_id,
                           PathTimeObstacle* path_time_obstacle);

  std::vector<std::pair<double, double>> GetPathBlockingIntervals(
      const double t) const;

  std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
      const double t_start, const double t_end, const double t_resolution);

  std::pair<double, double> get_path_range() const;

  std::pair<double, double> get_time_range() const;

  std::vector<PathTimePoint> GetObstacleSurroundingPoints(
      const int32_t& obstacle_id, const double s_dist,
      const double t_density) const;

  bool IsObstacleInGraph(const int32_t& obstacle_id);

  std::vector<std::pair<double, double>> GetLateralBounds(
      const double s_start, const double s_end, const double s_resolution);

 private:
  void SetupObstacles(
      const std::vector<const Obstacle*>& obstacles,
      const std::vector<PathPoint>& discretized_ref_points);

  SLBoundary ComputeObstacleBoundary(
      const std::vector<Vec2d>& vertices,
      const std::vector<PathPoint>& discretized_ref_points) const;

  PathTimePoint SetPathTimePoint(const int32_t& obstacle_id, const double s,
                                 const double t) const;
  void SetStaticObstacle(
      const Obstacle* obstacle,
      const std::vector<PathPoint>& discretized_ref_points);

  void SetDynamicObstacle(
      const Obstacle* obstacle,
      const std::vector<PathPoint>& discretized_ref_points);

  void UpdateLateralBoundsByObstacle(
    const SLBoundary& sl_boundary,
    const std::vector<double>& discretized_path,
    const double s_start, const double s_end,
    std::vector<std::pair<double, double>>* const bounds);

 private:
  std::pair<double, double> time_range_;
  std::pair<double, double> path_range_;
  const ReferenceLineInfo* ptr_reference_line_info_;
  std::array<double, 3> init_d_;
  std::unordered_map<uint32_t, PathTimeObstacle> path_time_obstacle_map_;
  std::vector<PathTimeObstacle> path_time_obstacles_;
  std::vector<SLBoundary> static_obs_sl_boundaries_;
};


} //end namespace planning

#endif // PATH_TIME_GRAPH_H
