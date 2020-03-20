

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "../../common/macro.h"
#include "../../math/cartesian_frenet_conversion.h"
#include "../../math/path_matcher.h"
#include "../../common/get_now_time.h"
#include "../../constraint_checker/collision_checker.h"
#include "../../constraint_checker/constraint_checker.h"
#include "../../lattice/behavior/path_time_graph.h"
#include "../../lattice/behavior/prediction_querier.h"
#include "../../lattice/trajectory1d/lattice_trajectory1d.h"
#include "../../lattice/trajectory_generation/backup_trajectory_generator.h"
#include "../../lattice/trajectory_generation/trajectory1d_generator.h"
#include "../../lattice/trajectory_generation/trajectory_combiner.h"
#include "../../lattice/trajectory_generation/trajectory_evaluator.h"
#include "../../planner/lattice/lattice_planner.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {

using common::Status;
using math::PathMatcher;
using math::CartesianFrenetConverter;


namespace {
#if 0
std::vector<PathPoint>
ToDiscretizedReferenceLine(const std::vector<ReferencePoint>& ref_points) {

  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.x=(ref_point.x());
    path_point.y=(ref_point.y());
    path_point.theta=(ref_point.heading());
    path_point.kappa=(ref_point.kappa());
    path_point.dkappa=(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x - path_points.back().x;
      double dy = path_point.y - path_points.back().y;
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.s=(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}
#endif
//s s' s'' L L' L'' 
void ComputeInitFrenetState(const PathPoint& matched_point,
                            const TrajectoryPoint& cartesian_state,
                            std::array<double, 3>* ptr_s,
                            std::array<double, 3>* ptr_d) {
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s, matched_point.x, matched_point.y,
      matched_point.theta, matched_point.kappa, matched_point.dkappa,
      cartesian_state.path_point.x, cartesian_state.path_point.y,
      cartesian_state.v, cartesian_state.a,
      cartesian_state.path_point.theta,
      cartesian_state.path_point.kappa, ptr_s, ptr_d);
}

}  // namespace


Status LatticePlanner::
Plan(const TrajectoryPoint& planning_start_point,Frame* frame)
{
  std::size_t success_line_count = 0;
  std::size_t index = 0;

  for (auto& reference_line_info : frame->reference_line_info())
  {

    if(index != 0)
        reference_line_info.SetPriorityCost(g_config_param.cost_non_priority_reference_line);

    if(!reference_line_info.IsSideSlipClean())
        reference_line_info.SetSlideCleanCost(g_config_param.cost_non_slide_reference_line);

    auto status =
       PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

    if (status != Status::OK) {
        if (reference_line_info.IsChangeLanePath()) {
          cout <<"Planner failed to change lane to "
               << reference_line_info.Lanes().Id()<<endl;
        } else {
          cout << "Planner failed to " << reference_line_info.Lanes().Id()<<endl;
        }
    } else {
      success_line_count += 1;
    }

    ++index;
  }

  if (success_line_count > 0) {
    return Status::OK;
  }else{
    cout<<"Failed to plan on any reference line."<<endl;
    return Status::PLANNING_ERROR; }
}

Status LatticePlanner::
PlanOnReferenceLine(const TrajectoryPoint& planning_init_point, Frame* frame,
                          ReferenceLineInfo* reference_line_info   )
{
  static std::size_t num_planning_cycles = 0;
  static std::size_t num_planning_succeeded_cycles = 0;
  unsigned long long start_time = Clock::NowInMs();
  unsigned long long current_time = start_time;

  cout << "Number of planning cycles: "   << num_planning_cycles <<endl;
  cout << "Number of planning succeeded: "<< num_planning_succeeded_cycles<<endl;

  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
  // 1. obtain a reference line and transform it to the PathPoint format.
  auto ptr_reference_line =
      std::make_shared< std::vector<PathPoint> >
          (reference_line_info->reference_line().reference_points());

  // 2. compute the matched point of the init planning point on the reference line.
    PathPoint matched_point = PathMatcher::MatchToPath(
        *ptr_reference_line, planning_init_point.path_point.x,
         planning_init_point.path_point.y);
#if 0
  auto& reference_line_provider = frame->reference_line_provider();
  PathPoint matched_point =
      reference_line_provider.AnalyticallyFindNearestRefPoint(
                planning_init_point.path_point.x,
                planning_init_point.path_point.y,
                reference_line_info->reference_line_mutable(), 1e-7, 50);

  int idx = matched_point.s / g_config_param.discretion_length;
  auto near_ref_point = ptr_reference_line.get()->at(idx);
  auto dx = matched_point.x - near_ref_point.x;
  auto dy = matched_point.y - near_ref_point.y;
  double dist = sqrt(dx * dx + dy * dy);
  matched_point.s = near_ref_point.s + dist;
#endif
  // 3. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
  cout<<"init_s = ("<<init_s[0]<<", "<<init_s[1]<<", "<<init_s[2]<<" )"<<endl;
  cout<<"init_d = ("<<init_d[0]<<", "<<init_d[1]<<", "<<init_d[2]<<" )"<<endl;
  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
                                      frame->obstacles(), ptr_reference_line);

  // 4. parse the decision and get the planning target.
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
       ptr_prediction_querier->GetObstacles(),
       *ptr_reference_line, reference_line_info,
       init_s[0] - 1.0, init_s[0] + g_config_param.decision_horizon,//= 100
       0.0, g_config_param.trajectory_time_length, init_d );   //= 8s

  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetCruiseSpeed(speed_limit);
  current_time = Clock::NowInMs();
  bool is_side_slip_clean = reference_line_info->IsSideSlipClean();
  auto sample_result = reference_line_info->SamepleDeciderWithObsSL(frame->obstacles());

  if(sample_result.pass_type == sample_result.STOP_DRIVR){
      double center_to_obs_stop_dis = g_config_param.valid_stop_decider_dis;
      if(!reference_line_info->Lanes().IsOnSegment())
          center_to_obs_stop_dis += 1.0;
      double stop_s = sample_result.sl_point.s - center_to_obs_stop_dis;
      if(stop_s < init_s[0]){
        cout<<"ego will stop if continue drive along this way ,so recommend stop"<<endl;
        reference_line_info->SetCost(30);
        return common::PLANNING_ERROR;
      }
  }

#if 0
 if(sample_result.pass_type == sample_result.RIGHT_PASS ){
     double right_to_center_dis = - g_config_param.sample_width;
     if(is_side_slip_clean)
         right_to_center_dis = 0;
     double kappa = ( init_d[0] - right_to_center_dis )/
                    ( abs(sample_result.sl_point.s - init_s[0]) +
                      g_config_param.lattice_epsilon );
     if( abs(kappa) > g_config_param.kappa_bound + 0.1 ){
        cout<<"ego kappa will bigger if continue drive along this way ,so recommend stop"<<endl;
        reference_line_info->SetCost(std::numeric_limits<double>::infinity());
        return common::PLANNING_ERROR;
     }
  }

 if(sample_result.pass_type == sample_result.LEFT_PASS){
     double left_to_center_dis = g_config_param.sample_width;
     if(is_side_slip_clean)
         left_to_center_dis *= 2;
     double kappa = ( init_d[0] - left_to_center_dis )/
                    ( abs(sample_result.sl_point.s - init_s[0]) +
                      g_config_param.lattice_epsilon );
     if( abs(kappa) > g_config_param.kappa_bound + 0.1 ){
        cout<<"ego kappa will bigger if continue drive along this way ,so recommend stop"<<endl;
        reference_line_info->SetCost(std::numeric_limits<double>::infinity());
        return common::PLANNING_ERROR;
     }

  }
#endif

  PlanningTarget planning_target = reference_line_info->planning_target();
  if (planning_target.has_stop_point) {
    cout << "Planning target stop s: " << planning_target.stop_point.s<<endl;
    cout << "Current ego s: " << init_s[0]<<endl;
  }

  cout<< "Decision_Time = " << (Clock::NowInMs() - current_time) <<endl;
  current_time = Clock::NowInMs();

  // 5. generate 1d trajectory bundle for longitudinal and lateral respectively.

  Trajectory1dGenerator trajectory1d_generator( init_s, init_d,
                                                ptr_path_time_graph,
                                                ptr_prediction_querier,
                                                is_side_slip_clean,
                                                sample_result
                                              );

  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;

  trajectory1d_generator.GenerateTrajectoryBundles( planning_target,
                                                   &lon_trajectory1d_bundle,
                                                   &lat_trajectory1d_bundle );
  cout << "Trajectory_Generation_Time = "
       << (Clock::NowInMs() - current_time)<<endl;
  current_time = Clock::NowInMs();

  // 6. first, evaluate the feasibility of the 1d trajectories according to dynamic
  // constraints.second, evaluate the feasible longitudinal and lateral trajectory
  // pairs and sort them according to the cost.
  auto garbages_sl = reference_line_info->GetGarbagesSL();
  TrajectoryEvaluator trajectory_evaluator( init_s,init_d, planning_target,
                                            lon_trajectory1d_bundle,
                                            lat_trajectory1d_bundle,
                                            ptr_path_time_graph,
                                            ptr_reference_line,
                                            garbages_sl,
                                            is_side_slip_clean
                                           );

  cout << "Trajectory_1d_Evaluator_Time = "
       << (Clock::NowInMs() - current_time)<<endl;
  current_time = Clock::NowInMs();

  cout << "number of trajectory pairs = "
       << trajectory_evaluator.num_of_trajectory_pairs() <<endl;
  cout << "  number_lon_traj = " << lon_trajectory1d_bundle.size()<<endl;
  cout << "  number_lat_traj = " << lat_trajectory1d_bundle.size()<<endl;

  // Get instance of collision checker and constraint checker
  CollisionChecker collision_checker( frame->obstacles(), init_s[0], init_d[0],
                                     *ptr_reference_line, reference_line_info,
                                      ptr_path_time_graph, frame->GetRouteType());

  // 7. always get the best pair of trajectories to combine; return the first
  // collision-free trajectory.
  std::size_t constraint_failure_count = 0;
  std::size_t collision_failure_count = 0;
  std::size_t combined_constraint_failure_count = 0;

  std::size_t lon_vel_failure_count = 0;
  std::size_t lon_acc_failure_count = 0;
  std::size_t lon_jerk_failure_count = 0;
  std::size_t curvature_failure_count = 0;
  std::size_t lat_acc_failure_count = 0;
  std::size_t lat_jerk_failure_count = 0;

  std::size_t num_lattice_traj = 0;

  while (trajectory_evaluator.has_more_trajectory_pairs())
  {

    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();//get mini pair cost

    // For auto tuning
    std::vector<double> trajectory_pair_cost_components;
    if (g_config_param.enable_auto_tuning) {
      trajectory_pair_cost_components =
          trajectory_evaluator.top_trajectory_pair_component_cost();
          //get mini pair cost 组成的元素项
      cout << "TrajectoryPairComponentCost"<<endl;
      cout << "1.objective_cost   = "<< trajectory_pair_cost_components[0]<<endl;
      cout << "2.jerk_cost        = "<< trajectory_pair_cost_components[1]<<endl;
      cout << "3.collision_cost   = "<< trajectory_pair_cost_components[2]<<endl;
      cout << "4.centripetal_cost = "<< trajectory_pair_cost_components[3]<<endl;
      cout << "5.offsets_cost     = "<< trajectory_pair_cost_components[4]<<endl;
      cout << "6.comfort_cost     = "<< trajectory_pair_cost_components[5]<<endl;
      cout << "7.clean_cost       = "<< trajectory_pair_cost_components[6]<<endl;

  }

    //获取最佳纵横向路径参数组合
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = TrajectoryCombiner::Combine(
                                  *ptr_reference_line,
                                  *trajectory_pair.first,
                                  *trajectory_pair.second,
                                  planning_init_point.relative_time,
                                  planning_target.has_stop_point);
    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
      case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
        lon_vel_failure_count += 1;
        break;
      case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
        lon_acc_failure_count += 1;
        break;
      case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
        lon_jerk_failure_count += 1;
        break;
      case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
        curvature_failure_count += 1;
        break;
      case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
        lat_acc_failure_count += 1;
        break;
      case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
        lat_jerk_failure_count += 1;
        break;
      case ConstraintChecker::Result::VALID:
      default:
        // Intentional empty
        break;
      }
      continue;
    }

    // check collision with other obstacles
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }

    // put combine trajectory into debug data
    const auto& combined_trajectory_points = combined_trajectory.trajectory_points();
    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost+
                                 reference_line_info->SlideCleanCost());
    reference_line_info->SetDrivable(true);

    // Print the chosen end condition and start condition
    cout << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1]
         << " dds = " << init_s[2]<<endl;
    // cast
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      cout << "Dynamically casting trajectory1d ptr. failed."<<endl;
    }

    if (lattice_traj_ptr->has_target_position()) {
      cout << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
             << " ds = " << lattice_traj_ptr->target_velocity()
             << " t = " << lattice_traj_ptr->target_time()<<endl;
    }

    cout << "InputPose";
    cout << "XY: " << planning_init_point.path_point.x<<","
                   << planning_init_point.path_point.y<<endl;
    cout << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2]
         << ")"<<endl;
    cout << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2]
         << ")"<<endl;
    cout<<"****************************************************************************************"<<endl<<endl;
    cout << "Reference_line_priority_cost = "
         << reference_line_info->PriorityCost()<<endl;
    cout << "Total_Trajectory_Cost = " << trajectory_pair_cost<<endl;
    cout << "OutputTrajectory"<<endl;
    for (uint i = 0; i < 4; ++i) {
      cout<<"point_"<<i<<": "<<std::setprecision(4)
          <<combined_trajectory_points[i].path_point.x<<","
          <<combined_trajectory_points[i].path_point.y<<","<<endl;
    }

    break;

  }

  cout << "Trajectory_Evaluation_Time = "
       << (Clock::NowInMs() - current_time)<<endl;

  cout << "Step CombineTrajectory Succeeded"<<endl;

  cout << "1d trajectory not valid for constraint ["
       << constraint_failure_count << "] times"<<endl;
  cout << "Combined trajectory not valid for ["
       << combined_constraint_failure_count << "] times"<<endl;
  cout << "curvature_failure_count times = "<<curvature_failure_count<<endl;
  cout << "Trajectory not valid for collision [" << collision_failure_count
       << "] times"<<endl;
  cout << "Total_Lattice_Planning_Frame_Time = "
       << (Clock::NowInMs() - start_time)<<" ms" <<endl;

  if (num_lattice_traj > 0) {
    cout << "Planning succeeded"<<endl<<endl;
    num_planning_succeeded_cycles += 1;
    reference_line_info->SetDrivable(true);
    return Status::OK;
  } else {
    cout << "Planning failed"<<endl;
    if (g_config_param.enable_backup_trajectory) {
      cout << "Use backup trajectory"<<endl;
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time,
          std::make_shared<CollisionChecker>(collision_checker),
          &trajectory1d_generator);

      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

      reference_line_info->AddCost(g_config_param.backup_trajectory_cost);//1000
      reference_line_info->SetTrajectory(trajectory);
      reference_line_info->SetDrivable(true);
      return Status::OK;

    } else {
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    }
    cout<<"No feasible trajectories"<<endl;
    return Status::PLANNING_ERROR;
  }
}

DiscretizedTrajectory LatticePlanner::GetFutureTrajectory() const {

  // localization
  const auto& localization =AdapterManager::GetLatestLocalization();
  //cout <<"Get localization: "<<endl;
  //cout <<"   x:"<< localization->x <<","<<endl;
  //cout <<"   y:"<< localization->y <<","<<endl;
  //cout <<"head:"<< localization->heading <<","<<endl;
  //cout <<"   v:"<< localization->linear_velocity <<","<<endl;
  //cout <<"   a:"<< localization->linear_acceleration <<","<<endl;

  std::vector<TrajectoryPoint> traj_pts;
//  for (const auto& traj_pt : localization.trajectory_point) {
//    traj_pts.emplace_back(traj_pt);
//  }
  DiscretizedTrajectory ret(traj_pts);
  return ret;
}

bool LatticePlanner::MapFutureTrajectoryToSL(
    const DiscretizedTrajectory& future_trajectory,
    const std::vector<PathPoint>& discretized_reference_line,
    std::vector<SpeedPoint>* st_points,
    std::vector<FrenetFramePoint>* sl_points) {
  if (0 == discretized_reference_line.size()) {
    cout << "MapFutureTrajectoryToSL error"<<endl;
    return false;
  }
  for (const TrajectoryPoint& trajectory_point :future_trajectory.trajectory_points())
    {
    const PathPoint& path_point = trajectory_point.path_point;
    PathPoint matched_point = PathMatcher::MatchToPath(
        discretized_reference_line, path_point.x, path_point.y);
    std::array<double, 3> pose_s;
    std::array<double, 3> pose_d;
    ComputeInitFrenetState(matched_point, trajectory_point, &pose_s, &pose_d);
    SpeedPoint st_point;
    FrenetFramePoint sl_point;
    st_point.s=(pose_s[0]);
    st_point.t=(trajectory_point.relative_time);
    st_point.v=(pose_s[1]);
    st_point.a=(pose_s[2]);  // Not setting da
    sl_point.s=(pose_s[0]);
    sl_point.l=(pose_d[0]);
    sl_point.dl=(pose_d[0]);
    sl_point.ddl=(pose_d[0]);
    st_points->emplace_back(std::move(st_point));
    sl_points->emplace_back(std::move(sl_point));
  }
  return true;
}

}  // namespace planning

