
/**
 * @file
 **/

#ifndef LATTICE_TRAJECTORY_GENERATION_TRAJECTORY1D_GENERATOR_H_
#define LATTICE_TRAJECTORY_GENERATION_TRAJECTORY1D_GENERATOR_H_

#include <memory>
#include <utility>
#include <vector>

#include "../../lattice/behavior/path_time_graph.h"
#include "../../lattice/behavior/prediction_querier.h"
#include "../../lattice/trajectory1d/lattice_trajectory1d.h"
#include "../../lattice/trajectory_generation/end_condition_sampler.h"
#include "../../math/curve1d/curve1d.h"
#include "../../math/curve1d/quartic_polynomial_curve1d.h"
#include "../../math/curve1d/quintic_polynomial_curve1d.h"


namespace planning {

class Trajectory1dGenerator {
 public:
  Trajectory1dGenerator(
      const std::array<double, 3>& lon_init_state,
      const std::array<double, 3>& lat_init_state,
      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier,
      const bool is_side_slip_clean,
      const SampleRecommendation sample_result);

  virtual ~Trajectory1dGenerator() = default;

  void GenerateTrajectoryBundles(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle);

  void GenerateLateralTrajectoryBundle(
      std::vector<std::shared_ptr<Curve1d>>* ptr_lat_trajectory_bundle) const;

 private:
  void GenerateSpeedProfilesForCruising(
      const double target_speed,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForStopping(
      const double stop_point,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateSpeedProfilesForPathTimeObstacles(
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

  void GenerateLongitudinalTrajectoryBundle(
      const PlanningTarget& planning_target,
      std::vector<std::shared_ptr<Curve1d>>* ptr_lon_trajectory_bundle) const;

template <std::size_t N>
    void GenerateTrajectory1DBundle(
        const std::array<double, 3>& init_state,
        const std::vector<std::pair<std::array<double, 3>, double>>&
            end_conditions,
        std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const;
    //void creat();

 private:
  std::array<double, 3> init_lon_state_;

  std::array<double, 3> init_lat_state_;

  EndConditionSampler end_condition_sampler_;

  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
};


template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<4>(
    const std::array<double, 3>& init_state,
    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const {
  //CHECK_NOTNULL(ptr_trajectory_bundle);
  //CHECK(!end_conditions.empty());

  ptr_trajectory_bundle->reserve(end_conditions.size());
  for (const auto& end_condition : end_conditions) {
    auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(
     std::shared_ptr<Curve1d>(new QuarticPolynomialCurve1d(
     init_state, {end_condition.first[1], end_condition.first[2]},
            end_condition.second)));

    ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
    ptr_trajectory1d->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory1d);
  }
}

template <>
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<5>(
    const std::array<double, 3>& init_state,
    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle ) const
{
  //CHECK_NOTNULL(ptr_trajectory_bundle);
  //CHECK(!end_conditions.empty());

  ptr_trajectory_bundle->reserve(end_conditions.size());

  for (const auto& end_condition : end_conditions)
  {
    auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(
        std::shared_ptr<Curve1d>(new QuinticPolynomialCurve1d(
            init_state, end_condition.first, end_condition.second)));

    ptr_trajectory1d->set_target_position(end_condition.first[0]);
    ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
    ptr_trajectory1d->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory1d);
  }
}


}  // namespace planning



#endif
// MODULES_PLANNING_LATTICE_TRAJECTORY_GENERATION_TRAJECTORY1D_GENERATOR_H_
