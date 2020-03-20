
/**
 * @file
 **/

#ifndef PLANNING_COMMON_PATH_DECISION_H_
#define PLANNING_COMMON_PATH_DECISION_H_

#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "../common/indexed_list.h"
#include "../common/obstacle.h"
#include "../common/path_obstacle.h"


namespace planning {

/**
 * @class PathDecision
 *
 * @brief PathDecision represents all obstacle decisions on one path.
 */
class PathDecision {
 public:
  PathDecision() = default;

  PathObstacle *AddPathObstacle(const PathObstacle &path_obstacle);

  const IndexedList<int32_t, PathObstacle> &path_obstacles() const;

  bool AddLateralDecision(const std::string &tag, const int32_t &object_id,
                          const ObjectDecisionType &decision);
  bool AddLongitudinalDecision(const std::string &tag,
                               const int32_t &object_id,
                               const ObjectDecisionType &decision);

  const PathObstacle *Find(const int32_t &object_id) const;

  PathObstacle *Find(const int32_t &object_id);

  void SetStBoundary(const int32_t &id, const StBoundary &boundary);
  void EraseStBoundaries();
  MainStop main_stop() const { return main_stop_; }
  double stop_reference_line_s() const { return stop_reference_line_s_; }
  bool MergeWithMainStop(const ObjectStop &obj_stop, const int32_t &obj_id,
                         const ReferenceLine &ref_line,
                         const SLBoundary &adc_sl_boundary);

 private:
  std::mutex obstacle_mutex_;
  IndexedList<int32_t, PathObstacle> path_obstacles_;
  MainStop main_stop_;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
};

}  // namespace planning


#endif  // PLANNING_COMMON_PATH_DECISION_H_
