


#include "../../lattice/trajectory_generation/trajectory1d_generator.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>


extern ConfigParam g_config_param;
namespace planning {

// A common function for trajectory bundles generation with
// a given initial state and  end conditions.
typedef std::array<double, 3> State;
typedef std::pair<State, double> Condition;
typedef std::vector<std::shared_ptr<Curve1d>> Trajectory1DBundle;

Trajectory1dGenerator::Trajectory1dGenerator(
    const State& lon_init_state,
    const State& lat_init_state,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier,
    const bool is_side_slip_clean,
    const SampleRecommendation sample_result)
    : init_lon_state_(lon_init_state),
      init_lat_state_(lat_init_state),
      end_condition_sampler_(lon_init_state, lat_init_state,
         ptr_path_time_graph, ptr_prediction_querier,is_side_slip_clean,sample_result),
      ptr_path_time_graph_(ptr_path_time_graph) {}

void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle,
    Trajectory1DBundle* ptr_lat_trajectory_bundle) {
  GenerateLongitudinalTrajectoryBundle(planning_target,
                                       ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
  return;
}
void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  // cruising trajectories are planned regardlessly.
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed,
                                   ptr_lon_trajectory_bundle   );

  GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);

  if (planning_target.has_stop_point) {
    GenerateSpeedProfilesForStopping(planning_target.stop_point.s,
                                     ptr_lon_trajectory_bundle );
  }
}
void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const
{
  cout << "cruise speed is  " << target_speed << endl;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);
  if (end_conditions.empty()) {
    cout << "cruise end_conditions is empty !!!" << endl;
    return;
  }

  // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
  // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
  // invoke the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
    const double stop_point,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
 cout << "stop point is " << stop_point<<endl;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForStopping(stop_point);
  if (end_conditions.empty()) {
    cout << "For Stopping end_conditions is empty " << endl;
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}

void Trajectory1dGenerator::GenerateSpeedProfilesForPathTimeObstacles(
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForPathTimePoints();
  if (end_conditions.empty()) {
    cout << "Path Time Obstacles end_conditions is empty " << endl;
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}


void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    Trajectory1DBundle* ptr_lat_trajectory_bundle) const {
  if (!g_config_param.lateral_optimization) {
    auto end_conditions = end_condition_sampler_.SampleLatEndConditions();

    // Use the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<5>(init_lat_state_, end_conditions,
                                  ptr_lat_trajectory_bundle);
  } else {
      cout<<"Error don't open lateral_optimization method"<<endl;
      return ;
//    double s_min = 0.0;
//    double s_max = g_config_param.max_s_lateral_optimization;//= 50

//    double delta_s = g_config_param.default_delta_s_lateral_optimization;// = 2.0

//    auto lateral_bounds =
//        ptr_path_time_graph_->GetLateralBounds(s_min, s_max, delta_s);//no using

//    LateralTrajectoryOptimizer lateral_optimizer;
//    LateralQPOptimizer lateral_optimizer;

//    lateral_optimizer.optimize(init_lat_state_, delta_s, lateral_bounds);

   // auto lateral_trajectory = lateral_optimizer.GetOptimalTrajectory();

    //ptr_lat_trajectory_bundle->push_back(
        //std::make_shared<PiecewiseJerkTrajectory1d>(lateral_trajectory));
  }
}

}  // namespace planning

