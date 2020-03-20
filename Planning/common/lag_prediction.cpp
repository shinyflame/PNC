
/**
 * @file
 **/

#include <iostream>
#include <algorithm>
#include "../math/util.h"
#include "../common/lag_prediction.h"
#include <unordered_set>
extern ConfigParam g_config_param;
using namespace std;

namespace planning {

using perception::PerceptionObstacle;
using prediction::PredictionObstacle;
using prediction::PredictionObstacles;

LagPrediction::LagPrediction(uint32_t min_appear_num,
                             uint32_t max_disappear_num)
    : min_appear_num_(min_appear_num),
      max_disappear_num_(max_disappear_num)
{
  if (g_config_param.prediction_message_history_limit <
      static_cast<int32_t>(min_appear_num_) )
  {
    cout  << "Prediction adapter history limit is "
          << g_config_param.prediction_message_history_limit
          << ", but an obstacle need to be observed at least "
          << min_appear_num_ << " times"<<endl;
    return;
  }
}

void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const
{

  obstacles->prediction_obstacle.clear();
  if (!AdapterManager::GetPrediction() ||
       AdapterManager::GetPredictionLists()->empty()) {
    return;
  }
  const auto &prediction = *(AdapterManager::GetPredictionLists());

  if (!AdapterManager::GetLocalization()) {  // no localization
    obstacles = AdapterManager::GetLatestObserved() ;//GetLatestObserved
    return;
  }

  const auto adc_position =
      AdapterManager::GetLatestLocalization().pose_30.back().position;

  const auto latest_prediction = &prediction.back();
  const double timestamp = latest_prediction->start_timestamp/1000.0;

  std::unordered_set<int> protected_obstacles;
  //1. manage the latest obstacles
  for (const auto& obstacle : latest_prediction->prediction_obstacle)
  {
    const auto& perception = obstacle.perception_obstacle;
    if (perception.confidence < g_config_param.perception_confidence_threshold &&
        perception.type != perception::VEHICLE) {
      continue;
    }

    double distance = math::util::DistanceXY(perception.position, adc_position);

    if (distance < g_config_param.lag_prediction_protection_distance) {// 30.0m
      protected_obstacles.insert(obstacle.perception_obstacle.id);
      // add protected obstacle
      AddObstacleToPrediction(0.0, obstacle, obstacles);
    }
  }

  std::unordered_map<int, LagInfo> obstacle_lag_info;
  int index = 0;  // data in begin() is the most recent data
  //2. manage history lag obstacle
  for (auto it = prediction.begin(); it != prediction.end(); ++it, ++index)
  {
    for (const auto& obstacle : (*it).prediction_obstacle) {
      const auto& perception = obstacle.perception_obstacle;
      auto id = perception.id;
      if (perception.confidence < g_config_param.perception_confidence_threshold &&
          perception.type != perception::VEHICLE) {
        continue;
      }
      if (protected_obstacles.count(id) > 0) {
        continue;  // don't need to count the already added protected obstacle
      }
      auto& info = obstacle_lag_info[id];
          ++info.count;
      if ((*it).start_timestamp/1000.0 > info.last_observed_time) {
        info.last_observed_time = (*it).start_timestamp/1000.0;
        info.last_observed_seq = index;
        info.obstacle_ptr = &obstacle;
      }
    }
  }

  //obstacles->header= latest_prediction->header;
  obstacles->start_timestamp = (latest_prediction->start_timestamp);
  obstacles->end_timestamp = (latest_prediction->end_timestamp);

  bool apply_lag = std::distance(prediction.begin(), prediction.end()) >=
                                      static_cast<int32_t>(min_appear_num_);
  for (const auto& iter : obstacle_lag_info)
  {
    if (apply_lag && iter.second.count < min_appear_num_) {
      continue;
    }
    if (apply_lag && iter.second.last_observed_seq <
        g_config_param.prediction_message_history_limit - max_disappear_num_) {
      continue;
    }
    if (iter.second.obstacle_ptr != nullptr) {
      AddObstacleToPrediction(timestamp - iter.second.last_observed_time,
                              *(iter.second.obstacle_ptr), obstacles);
    }
  }

}


void LagPrediction::AddObstacleToPrediction(double delay_sec,
                       const prediction::PredictionObstacle& history_obstacle,
                       prediction::PredictionObstacles* obstacles) const {

  auto obstacle = &obstacles->prediction_obstacle;
  if (obstacle == nullptr) {
    cout << "obstalce is nullptr."<<endl;
    return;
  }

  if (delay_sec <= 1e-6) {
    obstacle->push_back(history_obstacle);//history_obstacle;
    return;
  }

  prediction::PredictionObstacle temp_obstacle;
  temp_obstacle.perception_obstacle = history_obstacle.perception_obstacle;

  for (const auto& hist_trajectory : history_obstacle.trajectory) {

     prediction::Trajectory temp_trajectory;
     for (const auto& hist_point : hist_trajectory.trajectory_point)
      {
         if (hist_point.relative_time < delay_sec) {
           continue;
         }

         TrajectoryPoint temp_point;
         temp_point = hist_point;
         temp_point.relative_time = hist_point.relative_time - delay_sec;
         temp_trajectory.trajectory_point.push_back(temp_point);
      }

     if (temp_trajectory.trajectory_point.size() <= 0) {
       continue;
     }
     temp_trajectory.probability=(hist_trajectory.probability);
     temp_obstacle.trajectory.push_back(temp_trajectory);
  }


  temp_obstacle.timestamp =  history_obstacle.timestamp;
  temp_obstacle.predicted_period= history_obstacle.predicted_period;
  obstacle->push_back(temp_obstacle);
}

}  // namespace planning

