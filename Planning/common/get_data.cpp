
#include "./get_data.h"
#include <iostream>
#include <string>
#include "../common/get_now_time.h"
#include <math.h>

extern planning::ConfigParam g_config_param;

namespace planning {

PbTrajectory g_Latest_Trajectory;

namespace AdapterManager {

  PredictionObstacles   g_obstacles_;
  LocalizationEstimate  g_Localization;
  Chassis               g_chassis;
  TrafficLightDetection g_traffic_detection;
  bool                  g_get_localization_status = false;
  bool                  g_get_prediction_status = false;
  bool                  g_get_chassis_status = false;
  bool                  g_get_traffic_status = false;

  std::list<PredictionObstacles> obstacles_observed_queue_;
  CleanTarget g_clean_target_;
//******************************************************************************//
//*****************************  GetPrediction  ******************************//
//******************************************************************************//

bool GetPrediction()
 {
    return g_get_prediction_status;
 }

std::list<PredictionObstacles> * GetPredictionLists()
 {

    return &obstacles_observed_queue_;
 }

PredictionObstacles *GetLatestObserved()
{
  return &g_obstacles_;
}
CleanTarget  GetCleanTarget(){ return g_clean_target_ ; }
//******************************************************************************//
//*****************************  GetLocalization  ******************************//
//******************************************************************************//

bool GetLocalization() { return g_get_localization_status;   }
void UpdataLocalization(LocalizationEstimate latest_localiztion){

  g_Localization = latest_localiztion;
}
const LocalizationEstimate& GetLatestLocalization(){ return g_Localization; }


//******************************************************************************//
//*****************************  GetChassis  ***********************************//
//******************************************************************************//

bool GetChassis(){
   return g_get_chassis_status;
 }
const Chassis& GetLatestChassisObserved() {


   return g_chassis;
}

//******************************************************************************//
//*****************************  GetTrafficLight  ******************************//
//******************************************************************************//

bool  GetTrafficLightDetection(){

   return g_get_traffic_status;
 }
double GetTrafficLightDetectionDelaySec(){

   return (Clock::NowInMs() - g_traffic_detection.camera_timestamp)/1000.0;
 }

TrafficLightDetection GetTrafficLightDetectionLatestObserved(){


   return g_traffic_detection;
 }

//******************************************************************************//
//*****************************  GetPlanning  **********************************//
//******************************************************************************//

bool GetPlanning(){ return g_Latest_Trajectory.trajectory_points.size()>0;}

PbTrajectory GetLatestTrajectory() { return  g_Latest_Trajectory; }

void UndataLatestTrajectory(PbTrajectory *trajectory)
{
   g_Latest_Trajectory = *trajectory;
}
void ClearLatestTrajectory (){

  g_Latest_Trajectory.trajectory_points.clear();
}
PlanningStatus* GetPlanningStatus(){

  PlanningStatus planning_status;
  return &planning_status;
}

//******************************************************************************//
//*****************************  Initialized  **********************************//
//******************************************************************************//

bool GetRoutingResponse(){
   return false;
 }

bool Initialized( const PredictionObstacles   *obstacles_ptr,
                        PredictionObstacles   &obstacles_,

                  const LocalizationEstimate  *localization_ptr,
                        LocalizationEstimate  &localization_,

                  const Chassis               *chassis_ptr,
                        Chassis               &chassis_,

                  const TrafficLightDetection *traffic_detection_ptr,
                        TrafficLightDetection &traffic_detection_,

                  const CleanTarget           *clean_target_ptr,
                        CleanTarget           &clean_target_ ,

                  const hdmap::PncRoutes      *pnc_routes_ptr,
                        hdmap::PncRoutes      &pnc_routes     )
{
  hdmap::PointLLH map_original_llh ;
  if(pnc_routes_ptr == nullptr || pnc_routes_ptr->empty()) {

      cout<<"Error pnc routes data is empty !!!"<<endl;
      return false;
    } else {
      map_original_llh = pnc_routes_ptr->front().map_original_point.point_llh;
      cout<<"Info map original LLH = ("<<map_original_llh.lon<<" ,"
                                       <<map_original_llh.lat<<")"<<endl;
      map_original_llh.lat *= (M_PI / 180.0);
      map_original_llh.lon *= (M_PI / 180.0);
      pnc_routes = *pnc_routes_ptr;
    }


  if( localization_ptr == nullptr ) {

      g_get_localization_status = false;
      cout<<"localization is null !!!"<<endl;
      return false;
    } else {

      g_get_localization_status = true; 
      if(localization_ptr->pose_30.empty()) {
          g_get_localization_status = false;
          cout<<"localizations data num = 0 !!!"<<endl;
          return false;
       }
      localization_ = *localization_ptr;
      localization_.pose_30.back().has_data = true;


    if(!g_config_param.enable_record_debug)
      if(!CoordinateConvert::LlhToMapXyz( localization_.pose_30.back(),map_original_llh)){
          g_get_localization_status = false;
          cout<<"Error localizations data convert failed !!!"<<endl;
          return false;   
       }
    //GpsTimeToUnixTime(&localization_);

      g_Localization = *localization_ptr;
    }

  if( obstacles_ptr == nullptr )
    {
      g_get_prediction_status = false;
      cout<<"Prediction Obstacles is null !!!"<<endl;
      return false;
    }else{
      g_get_prediction_status = true;
      obstacles_ = *obstacles_ptr;
      auto match_pose = FindMatchLocation(obstacles_.start_timestamp,
                                          g_Localization,map_original_llh);
      double theta =   -match_pose.euler_angles.z;
      MakePredictionForObstacles(obstacles_,match_pose);
      CoordinateConvert::FromCarToMapPosition( obstacles_,
                                               match_pose.position, theta);
      g_obstacles_ = obstacles_;
      obstacles_observed_queue_.push_back(g_obstacles_);
      if(obstacles_observed_queue_.size() > 10){
          obstacles_observed_queue_.pop_front();
        }
    }

  if( chassis_ptr == nullptr )
    {
      g_get_chassis_status = false;
      cout<<"localization is null !!!"<<endl;
      return false;
    }else{
      g_get_chassis_status  = true;
      chassis_ = *chassis_ptr;
      chassis_.has_data = true;
      g_chassis = chassis_;
    }

  if( traffic_detection_ptr == nullptr )
    {
      g_get_traffic_status = false;
      cout<<"Error localization is null !"<<endl;
      return false;
    }else{
      g_get_traffic_status = true;
      traffic_detection_   = *traffic_detection_ptr;
      g_traffic_detection  = traffic_detection_;
    }

  if( clean_target_ptr == nullptr )
      {

        cout<<"Error clean_target_ptr is null !" <<endl;
        return false;
      }else{

        clean_target_ = *clean_target_ptr;
        Pose current_pose =  g_Localization.pose_30.back();
        double theta =  - current_pose.euler_angles.z;
        CoordinateConvert::FromCarToMapPosition(clean_target_ ,
                                                current_pose.position,
                                                theta  );
        g_clean_target_ = clean_target_;
      }

    return true;
}



Pose FindMatchLocation(uint64_t obs_timestamp,LocalizationEstimate  &localization_,
                       const hdmap::PointLLH &map_original_llh){


  uint32_t min_time_gap = 0;
  bool first = true;
  Pose match_pose;
  for(Pose pose:localization_.pose_30){
      uint32_t temp_time_gap;
      if(obs_timestamp > pose.timestamp_ms)
        temp_time_gap = obs_timestamp - pose.timestamp_ms;
      else
        temp_time_gap = pose.timestamp_ms - obs_timestamp;

      if(first){
          min_time_gap = temp_time_gap;
          first = false;
        }
      if(temp_time_gap <= min_time_gap){
          min_time_gap = temp_time_gap;
          match_pose = pose;
        }
      if(min_time_gap < 20) break;
    }


  if(!g_config_param.enable_record_debug)
      CoordinateConvert::LlhToMapXyz( match_pose,map_original_llh);

  return match_pose;

}

void MakePredictionForObstacles(PredictionObstacles &obstacles_,
                                const Pose &match_pose){

  for(auto & prediction_obstacle : obstacles_.prediction_obstacle )
    {

      prediction_obstacle.perception_obstacle.velocity.x += match_pose.linear_velocity.x;
      prediction_obstacle.perception_obstacle.velocity.y += match_pose.linear_velocity.y;
      double v_abs = sqrt(pow(prediction_obstacle.perception_obstacle.velocity.x,2) +
                          pow(prediction_obstacle.perception_obstacle.velocity.y,2));
      auto obs_velocity = prediction_obstacle.perception_obstacle.velocity;
      auto obs_position = prediction_obstacle.perception_obstacle.position;

      double obs_theta =
          prediction_obstacle.perception_obstacle.theta /180.0*M_PI - M_PI_2;
      if(obs_theta > M_PI){
          obs_theta -= 2.0*M_PI;
        }
      else if(obs_theta < -M_PI){
         obs_theta += 2.0*M_PI;
        }
      prediction_obstacle.perception_obstacle.theta = obs_theta;
      prediction_obstacle.trajectory.clear();
      continue;//cancel prediction
      if(v_abs < g_config_param.static_obstacle_speed_threshold){

          prediction_obstacle.trajectory_num = 0;
          prediction_obstacle.trajectory.clear();
          continue;
       }else{

          int traj_num = 1;
          int point_num = g_config_param.planning_make_prediction_max_points_num;
          prediction_obstacle.trajectory_num = traj_num;
          prediction_obstacle.trajectory.clear();

          prediction::Trajectory trajectory;
          trajectory.probability = 1;
          trajectory.trajectory_point_num = point_num;
          PathPoint v_point;
          v_point.x = obs_velocity.x;
          v_point.y = obs_velocity.y;
          perception::Point point_;
          CoordinateConvert::TranslationAndRotaion( v_point,point_,
                                                    match_pose.euler_angles.z);
          for(int i = 0 ; i < point_num ; i++){
              TrajectoryPoint trajectory_point;
              trajectory_point.relative_time = i * 0.1;
              trajectory_point.v = v_abs;
              trajectory_point.path_point.theta = obs_theta;             
              trajectory_point.path_point.x =obs_position.x + v_point.x * trajectory_point.relative_time;
              trajectory_point.path_point.y =obs_position.y + v_point.y * trajectory_point.relative_time;
              trajectory_point.path_point.s = v_abs * trajectory_point.relative_time;
              trajectory.trajectory_point.push_back(trajectory_point);
            }
          prediction_obstacle.trajectory.push_back(trajectory);
       }

    }

 }

bool GpsTimeToUnixTime(LocalizationEstimate * localization_ptr){

  uint64_t now_time_ms = Clock::NowInMs();
  uint64_t day_remainder = now_time_ms%(24*3600*1000);
  uint64_t day_round_ms = now_time_ms - day_remainder ;
  for(auto &pose : localization_ptr->pose_30){
      int week_ms = pose.gps_week_sec *1000;
      pose.timestamp_ms = day_round_ms + week_ms%(24*3600*1000);
    }
  return true;
}

} //namespace AdapterManager

} // namespace planning

