
/**
 * @file
 **/

#ifndef PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_
#define PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_

#include <string>
#include <vector>


#include "../../common/frame.h"
#include "../../common/reference_line_info.h"
#include "../../planner/planner.h"



namespace planning {

class LatticePlanner : public PlannerWithReferenceLine {
 public:
  LatticePlanner() = default;

  virtual ~LatticePlanner() = default;

  std::string Name() override { return "LATTICE"; }

  common::Status Init() override {

    cout<<"Lattice Initial Successful !!!"<<endl;
   return common::Status::OK;
   }

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(const TrajectoryPoint& planning_init_point,
                      Frame* frame) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status PlanOnReferenceLine(
      const TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;

 private:
  DiscretizedTrajectory GetFutureTrajectory() const;

  bool MapFutureTrajectoryToSL(
      const DiscretizedTrajectory& future_trajectory,
      const std::vector<PathPoint>& discretized_reference_line,
      std::vector<SpeedPoint>* st_points,
      std::vector<FrenetFramePoint>* sl_points);
};

}  // namespace planning


#endif  // MODULES_PLANNING_PLANNER_LATTICE_LATTICE_PLANNER_H_
