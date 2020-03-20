﻿
/**
 * @file
 **/

#ifndef PLANNING_COMMON_PATH_OBSTACLE_H_
#define PLANNING_COMMON_PATH_OBSTACLE_H_

#include <list>
#include <string>
#include <unordered_map>
#include <vector>

#include "../math/box2d.h"
#include "../math/vec2d.h"
#include "../common/indexed_list.h"
#include "../common/obstacle.h"
#include "../common/speed/st_boundary.h"
#include "../reference_line/reference_line.h"

namespace planning {

/**
 * @class PathObstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision saftey priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class PathObstacle {
 public:
  PathObstacle() = default;
  explicit PathObstacle(const Obstacle* obstacle);

  const int32_t &Id() const;

  const Obstacle* obstacle() const;

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType& LongitudinalDecision() const;

  const std::string DebugString() const;

  const SLBoundary& PerceptionSLBoundary() const;

  const StBoundary& reference_line_st_boundary() const;

  const StBoundary& st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const ObjectDecisionType& decision);
  bool HasLateralDecision() const;

  void SetStBoundary(const StBoundary& boundary);

  void SetStBoundaryType(const StBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const StBoundary& boundary);

  void SetReferenceLineStBoundaryType(const StBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  double MinRadiusStopDistance(const VehicleParam& vehicle_param) const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                    const double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  /**
   * @brief check if a ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

  /**
   * @brief check if a ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

 private:
  //FRIEND_TEST(MergeLongitudinalDecision, AllDecisions);
  static ObjectDecisionType MergeLongitudinalDecision(
      const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
  //FRIEND_TEST(MergeLateralDecision, AllDecisions);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs,
                                                 const ObjectDecisionType& rhs);

  bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                 const double adc_start_s,
                                 StBoundary* const st_boundary);
  bool IsValidObstacle(
      const perception::PerceptionObstacle& perception_obstacle);
  int32_t id_;
  const Obstacle* obstacle_ = nullptr;
  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  SLBoundary perception_sl_boundary_;

  StBoundary reference_line_st_boundary_;
  StBoundary st_boundary_;

  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  bool is_blocking_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;
/*
  struct ObjectTagCaseHash {
    std::size_t operator()(
        const ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<std::size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
*/
};

}  // namespace planning


#endif  // MODULES_PLANNING_COMMON_PATH_OBSTACLE_H_
