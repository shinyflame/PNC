


#ifndef END_CONDITION_SAMPLER_H
#define END_CONDITION_SAMPLER_H

#include <array>
#include <utility>
#include <vector>
#include <memory>
#include <string>

#include "../../common/struct.h"
#include "../../lattice/behavior/feasible_region.h"
#include "../../lattice/behavior//path_time_graph.h"
#include "../../lattice/behavior/prediction_querier.h"

namespace planning {

class EndConditionSampler {
 public:
  EndConditionSampler(
      const std::array<double, 3>& init_s,
      const std::array<double, 3>& init_d,
      std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
      std::shared_ptr<PredictionQuerier> ptr_prediction_querier,
      const bool is_side_slip_clean,
      const SampleRecommendation sample_result);

  virtual ~EndConditionSampler() = default;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLatEndConditions() const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForStopping(const double ref_stop_point) const;

  std::vector<std::pair<std::array<double, 3>, double>>
  SampleLonEndConditionsForPathTimePoints() const;

 private:
  std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

  void QueryFollowPathTimePoints(
      const VehicleParam & vehicle_config,
      const int32_t& obstacle_id,
      std::vector<SamplePoint>* sample_points) const;

  void QueryOvertakePathTimePoints(
      const VehicleParam& vehicle_config,
      const int32_t& obstacle_id,
      std::vector<SamplePoint>* sample_points) const;

 private:
  std::array<double, 3> init_s_;
  std::array<double, 3> init_d_;
  FeasibleRegion feasible_region_;
  std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
  std::shared_ptr<PredictionQuerier> ptr_prediction_querier_;
  bool is_side_slip_clean_ ;
  SampleRecommendation sample_result_;
};

}//namespace planning

#endif // END_CONDITION_SAMPLER_H
