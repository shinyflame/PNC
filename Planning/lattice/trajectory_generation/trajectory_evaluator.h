
/**
 * @file
 **/

#ifndef PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY_EVALUATOR_H_
#define PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY_EVALUATOR_H_

#include <array>
#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>
#include "../../common/struct.h"
#include "../../lattice/behavior/path_time_graph.h"
#include "../../math/curve1d/curve1d.h"

namespace planning {

class TrajectoryEvaluator {
  // normal use
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>, double>
      PairCost;

  // auto tuning
  typedef std::pair<
      std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>,
      std::pair<std::vector<double>, double>>
      PairCostWithComponents;

 public:
  explicit TrajectoryEvaluator(
      const std::array<double, 3>& init_s,
      const std::array<double, 3>& init_l,
      const PlanningTarget& planning_target,
      const std::vector<std::shared_ptr<Curve1d>>& lon_trajectories,
      const std::vector<std::shared_ptr<Curve1d>>& lat_trajectories,
      std::shared_ptr<PathTimeGraph> path_time_graph,
      std::shared_ptr<std::vector<PathPoint>> reference_line,
      const vector<SLPoint>& garbages_sl,
      const bool is_slide_clean);

  virtual ~TrajectoryEvaluator() = default;

  bool has_more_trajectory_pairs() const;

  std::size_t num_of_trajectory_pairs() const;

  std::pair<std::shared_ptr<Curve1d>, std::shared_ptr<Curve1d>>
  next_top_trajectory_pair();

  double top_trajectory_pair_cost() const;

  std::vector<double> top_trajectory_pair_component_cost() const;

  double EvaluateDiscreteTrajectory(
      const PlanningTarget& planning_target,
      const std::vector<SpeedPoint>& st_points,
      const std::vector<FrenetFramePoint>& sl_points,
      std::vector<double>* cost_components);

 private:
  double Evaluate(const PlanningTarget& planning_target,
                  const std::shared_ptr<Curve1d>& lon_trajectory,
                  const std::shared_ptr<Curve1d>& lat_trajectory,
                  std::vector<double>* cost_components = nullptr) const;

  double LonCostEvalute(const PlanningTarget& planning_target,
                        const std::shared_ptr<Curve1d>& lon_trajectory,
                        std::vector<double>* cost_components = nullptr) const;

  double LatCostEvalute( const std::shared_ptr<Curve1d>& lon_trajectory,
                         const std::shared_ptr<Curve1d>& lat_trajectory,
                         std::vector<double>* cost_components = nullptr) const;

  double LatOffsetCost(const std::shared_ptr<Curve1d>& lat_trajectory,
                       const std::vector<double>& s_values) const;

  double LatOffsetCost(
      const std::vector<FrenetFramePoint> sl_points) const;

  double LatComfortCost(const std::shared_ptr<Curve1d>& lon_trajectory,
                        const std::shared_ptr<Curve1d>& lat_trajectory) const;

  double LatComfortCost(
      const std::vector<FrenetFramePoint>& sl_points) const;
  ///clean garbages cost
  double CleanCompleteCost(const std::shared_ptr<Curve1d>& lat_trajectory,
                           const vector<SLPoint>& sl_points,
                           const double &s0,
                           const vector<double> &s_value)const;

  double LonComfortCost(const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double LonComfortCost(
      const std::vector<SpeedPoint>& st_points) const;

  double LonCollisionCost(const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double LonCollisionCost(
      const std::vector<SpeedPoint>& st_points) const;

  double LonObjectiveCost(const std::shared_ptr<Curve1d>& lon_trajectory,
                          const PlanningTarget& planning_target,
                          const std::vector<double>& ref_s_dot) const;

  double LonObjectiveCost(
      const std::vector<SpeedPoint>& st_points,
      const PlanningTarget& planning_target,
      const std::vector<double>& ref_s_dots) const;

  double CentripetalAccelerationCost(
      const std::shared_ptr<Curve1d>& lon_trajectory) const;

  double CentripetalAccelerationCost(
      const std::vector<SpeedPoint>& st_points) const;

  std::vector<double> ComputeLongitudinalGuideVelocity(
      const PlanningTarget& planning_target) const;

  bool InterpolateDenseStPoints(
      const std::vector<SpeedPoint>& st_points,
      double t, double *traj_s) const;

  struct CostComparator
      : public std::binary_function<const PairCost&, const PairCost&, bool> {
    bool operator()(const PairCost& left, const PairCost& right) const {
      return left.second > right.second;
    }
  };

  struct CostComponentComparator
      : public std::binary_function<const PairCostWithComponents&,
                                    const PairCostWithComponents&, bool> {
    bool operator()(const PairCostWithComponents& left,
                    const PairCostWithComponents& right) const {
      return left.second.second > right.second.second;
    }
  };

  std::priority_queue<PairCost, std::vector<PairCost>, CostComparator> cost_queue_;

  std::priority_queue< PairCostWithComponents,std::vector<PairCostWithComponents>,
                       CostComponentComparator > cost_queue_with_components_;

  std::shared_ptr<PathTimeGraph> path_time_graph_;

  std::shared_ptr<std::vector<PathPoint>> reference_line_;

  std::vector<std::vector<std::pair<double, double>>> path_time_intervals_;

  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;

  std::vector<double> reference_s_dot_;

  vector<SLPoint> garbages_sl_;
  bool is_slide_clean_;
};

}  // namespace planning


#endif
// MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY_EVALUATOR_H_
