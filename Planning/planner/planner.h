
#ifndef MODULES_PLANNING_PLANNER_PLANNER_H_
#define MODULES_PLANNING_PLANNER_PLANNER_H_

#include <string>
#include "../common/frame.h"
#include "../common/struct.h"

/**
 * @namespace planning
 * @brief planning
 */

namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */

class Planner {
 public:
  /**
   * @brief Constructor
   */
  Planner() = default;

  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  virtual std::string Name() = 0;
  virtual common::Status Init() = 0;

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual common::Status Plan(
      const TrajectoryPoint& planning_init_point, Frame* frame) = 0;

 protected:
  //PlanningConfig config_;
  //ScenarioManager scenario_manager_;
  //Scenario* scenario_;
};

class PlannerWithReferenceLine : public Planner {
 public:
  /**
   * @brief Constructor
   */
  PlannerWithReferenceLine() = default;

  /**
   * @brief Destructor
   */
  virtual ~PlannerWithReferenceLine() = default;

  /**
   * @brief Compute a trajectory for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual common::Status PlanOnReferenceLine(
      const TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) {
    //CHECK_NOTNULL(frame);
    return common::Status::OK;
  }
};

}  // namespace planning


#endif  // MODULES_PLANNING_PLANNER_PLANNER_H_
