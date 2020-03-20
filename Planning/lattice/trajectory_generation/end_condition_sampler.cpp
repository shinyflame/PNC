

#include <algorithm>
#include <utility>
#include "../../lattice/trajectory_generation/end_condition_sampler.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

EndConditionSampler::EndConditionSampler(
    const State& init_s, const State& init_d,
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier,
    const bool is_side_slip_clean,
    const SampleRecommendation sample_result)
    : init_s_(init_s),
      init_d_(init_d),
      feasible_region_(init_s),
      ptr_path_time_graph_(std::move(ptr_path_time_graph)),
      ptr_prediction_querier_(std::move(ptr_prediction_querier)),
      is_side_slip_clean_(is_side_slip_clean),
      sample_result_(sample_result){}


std::vector<Condition> EndConditionSampler::SampleLatEndConditions() const {
  std::vector<Condition> end_d_conditions;


  vector<double> end_d_candidates ;
  double sample_width = g_config_param.sample_width;
  double slide_trans_dis = g_config_param.slide_trans_dis;
  double max_sample_width = g_config_param.sample_width * 2.0;

  if(sample_result_.pass_type == sample_result_.LEFT_PASS){
     if(is_side_slip_clean_){
        end_d_candidates = {sample_width,max_sample_width};
     }else{
        end_d_candidates = {sample_width};
     }
  } else if(sample_result_.pass_type == sample_result_.RIGHT_PASS) {
     if(is_side_slip_clean_){
        end_d_candidates = {-0.2,0.0+slide_trans_dis};
     }else{
        end_d_candidates = {-sample_width};
     }
  } else if(sample_result_.pass_type == sample_result_.LEFT_CENTER_PASS) {
      if(is_side_slip_clean_){
         end_d_candidates = {max_sample_width,sample_width};
      }else{
         end_d_candidates = {0.0,sample_width};
      }
   } else if(sample_result_.pass_type == sample_result_.RIGHT_CENTER_PASS) {
      if(is_side_slip_clean_){
         end_d_candidates = {0.0+slide_trans_dis,sample_width};
      }else{
         end_d_candidates = {0.0,-sample_width};
      }
   } else if(sample_result_.pass_type == sample_result_.LEFT_RIGHT) {
      if(is_side_slip_clean_){
         end_d_candidates = {0.0+slide_trans_dis,max_sample_width};
      }else{
         end_d_candidates = {-sample_width,sample_width};
      }
   } else{if(is_side_slip_clean_){
         end_d_candidates = {0.0+slide_trans_dis,sample_width,max_sample_width};
      }else{
         end_d_candidates = {-sample_width,0.0,sample_width};
      }
  }


  std::array<double, 4> end_s_candidates = {3,5,8,12};

  for (const auto& s : end_s_candidates) {
    for (const auto& d : end_d_candidates) {
      State end_d_state = {d, 0.0, 0.0};
      Condition end_d_condition(end_d_state, s);
      end_d_conditions.emplace_back(end_d_condition);
    }
  }
  return end_d_conditions;
}
//max sample time is 4*9 = 36
std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const {
  //CHECK_GT(FLAGS_num_velocity_sample, 1);

  // time interval is one second //plus the last one 0.01
  constexpr std::size_t num_of_time_samples = 9;
  std::array<double, num_of_time_samples> time_samples;
  for (std::size_t i = 0; i + 1 < num_of_time_samples; ++i) {
    time_samples[i] = g_config_param.trajectory_time_length - i;
  }
  time_samples[num_of_time_samples - 1] = g_config_param.polynomial_minimal_param;//0.01s

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_samples) {//1 2 3 4 5 6 7 8 0.01
    //according max accerlation and time caculate v_upper
    double v_upper = std::min(feasible_region_.VUpper(time), ref_cruise_speed);
    //according max deceleration caculate v_lower
    double v_lower = feasible_region_.VLower(time);

    State lower_end_s = {0.0, v_lower, 0.0};
    Condition end_d_condition1(lower_end_s, time);
    end_s_conditions.emplace_back(end_d_condition1);

    State upper_end_s = {0.0, v_upper, 0.0};
    Condition end_d_condition2(upper_end_s, time);
    end_s_conditions.emplace_back(end_d_condition2);

    double v_range = v_upper - v_lower;
    // Number of sample velocities
    std::size_t num_of_mid_points = std::min(
      static_cast<std::size_t>(g_config_param.num_velocity_sample - 2),// 6
      static_cast<std::size_t>(v_range / g_config_param.min_velocity_sample_gap));//1 m/s

    if (num_of_mid_points > 0)
    {
      double velocity_seg = v_range / (num_of_mid_points + 1);
      for (std::size_t i = 1; i <= num_of_mid_points; ++i) {
        State end_s = {0.0, v_lower + velocity_seg * i, 0.0};
        Condition end_d_condition3(end_s, time);
        end_s_conditions.emplace_back(end_d_condition3);
      }
    }

  }
  return end_s_conditions;
}
//for stopping smaple the reference stop point distance is const but time is different
std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForStopping(
    const double ref_stop_point) const {
  // time interval is one second plus the last one 0.01
  constexpr std::size_t num_time_section = 9;
  std::array<double, num_time_section> time_sections;
  for (std::size_t i = 0; i + 1 < num_time_section; ++i) {
    time_sections[i] = g_config_param.trajectory_time_length - i;
  }
  time_sections[num_time_section - 1] = g_config_param.polynomial_minimal_param;//0.01s

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_sections) {
    State end_s = {std::max(init_s_[0], ref_stop_point), 0.0, 0.0};
    Condition end_d_condition(end_s, time);
    end_s_conditions.emplace_back(end_d_condition);
  }
  return end_s_conditions;
}

std::vector<Condition>
EndConditionSampler::SampleLonEndConditionsForPathTimePoints() const {
  std::vector<Condition> end_s_conditions;

  std::vector<SamplePoint> sample_points = QueryPathTimeObstacleSamplePoints();
  for (const SamplePoint& sample_point : sample_points) {
    if (sample_point.path_time_point.t <
                           g_config_param.polynomial_minimal_param) {//0.01s
      continue;
    }
    double s = sample_point.path_time_point.s;
    double v = sample_point.ref_v;
    double t = sample_point.path_time_point.t;
    if (s > feasible_region_.SUpper(t) || s < feasible_region_.SLower(t)) {
      continue;
    }
    State end_state = {s, v, 0.0};
    Condition end_d_condition(end_state, t);
    end_s_conditions.emplace_back(end_d_condition);
  }
  return end_s_conditions;
}

std::vector<SamplePoint>
EndConditionSampler::QueryPathTimeObstacleSamplePoints() const {
  const auto& vehicle_config = g_vehicle_config;
      //common::VehicleConfigHelper::instance()->GetConfig();
  std::vector<SamplePoint> sample_points;
  for (const auto& path_time_obstacle :ptr_path_time_graph_->GetPathTimeObstacles())
  {
    int32_t obstacle_id = path_time_obstacle.obstacle_id;
    QueryFollowPathTimePoints  (vehicle_config, obstacle_id, &sample_points);
    QueryOvertakePathTimePoints(vehicle_config, obstacle_id, &sample_points);
  }
  return sample_points;
}

void EndConditionSampler::QueryFollowPathTimePoints(
    const VehicleParam& vehicle_config,
    const int32_t& obstacle_id,
    std::vector<SamplePoint>* const sample_points) const
{
  std::vector<PathTimePoint> follow_path_time_points =
  ptr_path_time_graph_->GetObstacleSurroundingPoints(
  obstacle_id, -g_config_param.lattice_epsilon, g_config_param.time_min_density);
                                                        //time_min_density = 1.0
  for (const auto& path_time_point : follow_path_time_points)
  {
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
        obstacle_id, path_time_point.s, path_time_point.t);
    // Generate candidate s
    double s_upper = path_time_point.s -vehicle_config.front_edge_to_center;
    double s_lower = s_upper - g_config_param.default_lon_buffer;//5.0
    //CHECK_GE(FLAGS_num_sample_follow_per_timestamp, 2);
    double s_gap = g_config_param.default_lon_buffer
        / static_cast<double>(g_config_param.num_sample_follow_per_timestamp - 1);
    //num_sample_follow_per_timestamp = 3
    for (std::size_t i = 0; i < g_config_param.num_sample_follow_per_timestamp; ++i)
    {
      double s = s_lower + s_gap * static_cast<double>(i);
      SamplePoint sample_point;
      sample_point.path_time_point = path_time_point;
      sample_point.path_time_point.s = s;
      sample_point.ref_v = v;//v = 0.0 m/s
      sample_points->push_back(std::move(sample_point));
    }
  }
}

void EndConditionSampler::QueryOvertakePathTimePoints(
    const VehicleParam& vehicle_config,
    const int32_t& obstacle_id,
    std::vector<SamplePoint>* sample_points) const
{
    std::vector<PathTimePoint> overtake_path_time_points =
    ptr_path_time_graph_->GetObstacleSurroundingPoints(
        obstacle_id, g_config_param.lattice_epsilon, g_config_param.time_min_density);

  for (const auto& path_time_point : overtake_path_time_points) {
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(
        obstacle_id, path_time_point.s, path_time_point.t);
    SamplePoint sample_point;
    sample_point.path_time_point = path_time_point;
    sample_point.path_time_point.s= path_time_point.s +
        g_config_param.default_lon_buffer + vehicle_config.back_edge_to_center;//modify
    sample_point.ref_v = v;
    sample_points->push_back(std::move(sample_point));
  }
}
