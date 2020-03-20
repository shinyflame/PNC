
#include "planning_config_param_initial.h"
#include "iostream"



string trajectory_flie_path = "/home/sxl/Planning/config/trajectorys.json";
string obstacles_flie_path  = "/home/sxl/Planning/config/obstacles.json";

namespace planning {



#if 1
bool ReadObstacles(string flie_path,
                   prediction::PredictionObstacles & prediction_obstacles){
  Json::Reader reader;
  Json::Value root;

  //从文件中读取，保证当前文件有test.json文件
  ifstream in(flie_path,ios::binary);
  //in.open("test.json", ios::binary);

 if(!in.is_open()){
     cout << "Error opening obstacles.json file\n";
     return false;
   }

 if(reader.parse(in,root))
 {
   for(unsigned int i = 0; i < root["prediction_obstacle"].size(); i++)
   {
       cout<<"i:"<<i<<endl;
       prediction::PredictionObstacle prediction_obstacle ;//repeated

       prediction_obstacle.perception_obstacle.id
           = root["prediction_obstacle"][i]["perception_obstacle"]["id"].asInt();
       prediction_obstacle.perception_obstacle.position.x
           = root["prediction_obstacle"][i]["perception_obstacle"]["position"]["x"].asDouble();
       prediction_obstacle.perception_obstacle.position.y
           = root["prediction_obstacle"][i]["perception_obstacle"]["position"]["y"].asDouble();
       prediction_obstacle.perception_obstacle.position.z
           = root["prediction_obstacle"][i]["perception_obstacle"]["position"]["z"].asDouble();
       prediction_obstacle.perception_obstacle.theta
           = root["prediction_obstacle"][i]["perception_obstacle"]["theta"].asDouble();
       prediction_obstacle.perception_obstacle.velocity.x
           = root["prediction_obstacle"][i]["perception_obstacle"]["velocity"]["x"].asDouble();
       prediction_obstacle.perception_obstacle.velocity.y
           = root["prediction_obstacle"][i]["perception_obstacle"]["velocity"]["y"].asDouble();
       prediction_obstacle.perception_obstacle.length
           = root["prediction_obstacle"][i]["perception_obstacle"]["length"].asDouble();
       prediction_obstacle.perception_obstacle.width
           = root["prediction_obstacle"][i]["perception_obstacle"]["width"].asDouble();
       prediction_obstacle.perception_obstacle.height
           = root["prediction_obstacle"][i]["perception_obstacle"]["height"].asDouble();

       for(int j = 0;
           j < root["prediction_obstacle"][i]["perception_obstacle"]["polygon_points"].size(); j++){
           perception::Point point;
           point.x =
           root["prediction_obstacle"][i]["perception_obstacle"]
               ["polygon_points"][j]["polygon_point"]["x"].asDouble();
           point.y =
           root["prediction_obstacle"][i]["perception_obstacle"]
               ["polygon_points"][j]["polygon_point"]["y"].asDouble();
           point.z =
           root["prediction_obstacle"][i]["perception_obstacle"]
               ["polygon_points"][j]["polygon_point"]["z"].asDouble();
           prediction_obstacle.perception_obstacle.polygon_point.push_back(point);
           cout<<"point_y"<<point.y<<endl;
           cout<<"j:"<<j<<endl;
         }

      prediction_obstacle.timestamp
          = root["prediction_obstacle"][i]["timestamp"].asDouble();
      prediction_obstacle.predicted_period
          = root["prediction_obstacle"][i]["tpredicted_period"].asDouble();

      for(int m = 0; m < root["prediction_obstacle"][i]["trajectorys_"].size(); m++ )
        {
          prediction::Trajectory trajectory_;
          TrajectoryPoint trajectory_point_;
          cout<<"m:"<<m<<endl;

          trajectory_.probability =
          root["prediction_obstacle"][i]["trajectorys_"][m]["trajectory"]["probability"].asDouble();
          cout<<"trajectory_.probability: "<<trajectory_.probability<<endl;
          for(int n = 0;
              n < root["prediction_obstacle"][i]["trajectorys_"][m]
              ["trajectory"]["trajectory_points"].size(); n++)
            {
              cout<<"n:"<<n<<endl;
              trajectory_point_.path_point.x =
              root["prediction_obstacle"][i]["trajectorys_"][m]["trajectory"]["trajectory_points"]
                  [n]["trajectory_point"]["path_point"]["x"].asDouble();
              trajectory_point_.path_point.y =
              root["prediction_obstacle"][i]["trajectorys_"][m]["trajectory"]["trajectory_points"]
                  [n]["trajectory_point"]["path_point"]["y"].asDouble();
              trajectory_point_.path_point.theta =
              root["prediction_obstacle"][i]["trajectorys_"][m]["trajectory"]["trajectory_points"]
                  [n]["trajectory_point"]["path_point"]["theta"].asDouble();
              trajectory_point_.v =
              root["prediction_obstacle"][i]["trajectorys_"][m]["trajectory"]["trajectory_points"]
                  [n]["trajectory_point"]["v"].asDouble();
              trajectory_point_.relative_time =
              root["prediction_obstacle"][i]["trajectorys_"][m]["trajectory"]["trajectory_points"]
                  [n]["trajectory_point"]["relative_time"].asDouble();
              trajectory_.trajectory_point.push_back(trajectory_point_);
              cout<<"v: "<<trajectory_point_.v<<endl;
            }

          prediction_obstacle.trajectory.push_back(trajectory_);
        }



     //cout<<"id:"<<prediction_obstacle.perception_obstacle.id<<endl;
     //cout<<"theta:"<<prediction_obstacle.perception_obstacle.theta<<endl;
     cout<<" "<<endl;
     prediction_obstacles.prediction_obstacle.push_back(prediction_obstacle);
  }
   cout << "Reading Obstacles Complete!" << endl;
   in.close();
   if(prediction_obstacles.prediction_obstacle.size() >0){
       cout<<"obstacles num: "<<prediction_obstacles.prediction_obstacle.size()<<endl;
       //cout<<trajectory_points.at(0).relative_time<<endl;
       //cout<<trajectory_points.end()->relative_time<<endl;
     }
   return true;
 }
 cout << "Reading Obstacles Error!" << endl;
 return false;
}

bool ReadTrajectory(string flie_path,
                    std::vector<TrajectoryPoint>& trajectory_points){
  Json::Reader reader;
  Json::Value root;

  //从文件中读取，保证当前文件有test.json文件
  ifstream in(flie_path,ios::binary);
  //in.open("test.json", ios::binary);

 if(!in.is_open()){
     cout << "Error opening file\n";
     return false;
   }
 TrajectoryPoint trajectory_point_;
 if(reader.parse(in,root))
 {
   for(unsigned int i = 0; i < root["trajectory_points_"].size(); i++)
   {
     trajectory_point_.path_point.x      = root["trajectory_points_"][i]["path_point"]["x"].asDouble();
     trajectory_point_.path_point.y      = root["trajectory_points_"][i]["path_point"]["y"].asDouble();
     trajectory_point_.path_point.z      = root["trajectory_points_"][i]["path_point"]["z"].asDouble();
     trajectory_point_.path_point.theta  = root["trajectory_points_"][i]["path_point"]["theta"].asDouble();
     trajectory_point_.path_point.kappa  = root["trajectory_points_"][i]["path_point"]["kappa"].asDouble();
     trajectory_point_.path_point.s      = root["trajectory_points_"][i]["path_point"]["s"].asDouble();
     trajectory_point_.path_point.dkappa = root["trajectory_points_"][i]["path_point"]["dkappa"].asDouble();
     trajectory_point_.path_point.ddkappa = root["trajectory_points_"][i]["path_point"]["ddkappa"].asDouble();
     trajectory_point_.v                 = root["trajectory_points_"][i]["v"].asDouble();
     trajectory_point_.a                 = root["trajectory_points_"][i]["a"].asDouble();
     trajectory_point_.relative_time     = root["trajectory_points_"][i]["relative_time"].asDouble();

     cout<<"i:"<<i<<", s = "<<trajectory_point_.path_point.s<<endl;
     trajectory_points.push_back(trajectory_point_);
  }
   cout << "Reading Complete!" << endl;
   in.close();
   if(trajectory_points.size() >0){
       cout<<trajectory_points.size()<<endl;
       cout<<trajectory_points.at(0).relative_time<<endl;
       cout<<trajectory_points.end()->relative_time<<endl;
     }
   return true;
 }
 cout << "Reading Error!" << endl;
 return false;
}
#endif

bool ReadFile(string flie_path,ConfigParam & planing_config)
{

  Json::Reader reader;
  Json::Value root;

  //从文件中读取，保证当前文件有test.json文件
  ifstream in(flie_path,ios::binary);
  //in.open("test.json", ios::binary);

  if(!in.is_open()){
     cout << "Error opening file\n";
     return false;
   }

  if(reader.parse(in,root)){

      //string name = root["name"].asString();
      //读取子节点信息
      if(
         ReadData(planing_config.speed_lower_bound, "01.speed_lower_bound",root)&&
         ReadData(planing_config.speed_upper_bound, "02.speed_upper_bound",root)&&
         ReadData(planing_config.longitudinal_acceleration_lower_bound,
                             "03.longitudinal_acceleration_lower_bound",root)&&
         ReadData(planing_config.longitudinal_acceleration_upper_bound,
                             "04.longitudinal_acceleration_upper_bound",root)&&
         ReadData(planing_config.lateral_acceleration_bound,"05.lateral_acceleration_bound",root)&&
         ReadData(planing_config.trajectory_time_length,    "06.trajectory_time_length",root)&&
         ReadData(planing_config.polynomial_minimal_param,  "07.polynomial_minimal_param",root)&&
         ReadData(planing_config.num_velocity_sample,       "08.num_velocity_sample",root)&&
         ReadData(planing_config.min_velocity_sample_gap,   "09.min_velocity_sample_gap",root)&&
         ReadData(planing_config.lattice_epsilon,           "10.lattice_epsilon",root)&&
         ReadData(planing_config.time_min_density,          "11.time_min_density",root)&&
         ReadData(planing_config.default_lon_buffer,        "12.default_lon_buffer",root)&&
         ReadData(planing_config.num_sample_follow_per_timestamp,
                             "13.num_sample_follow_per_timestamp",root)&&
         ReadData(planing_config.use_navigation_mode,       "14.use_navigation_mode",root)&&
         ReadData(planing_config.static_obstacle_speed_threshold,
                             "15.static_obstacle_speed_threshold",root)&&
         ReadData(planing_config.virtual_stop_wall_height,    "16.virtual_stop_wall_height",root)&&
         ReadData(planing_config.default_reference_line_width,"17.default_reference_line_width",root)&&
         ReadData(planing_config.trajectory_time_resolution,  "18.trajectory_time_resolution",root)&&
         ReadData(planing_config.bound_buffer,"19.bound_buffer",root)&&
         ReadData(planing_config.nudge_buffer,"20.nudge_buffer",root)&&
         ReadData(planing_config.lateral_optimization,"21.lateral_optimization",root)&&
         ReadData(planing_config.max_s_lateral_optimization,"22.max_s_lateral_optimization",root)&&
         ReadData(planing_config.default_delta_s_lateral_optimization,
                             "23.default_delta_s_lateral_optimization",root)&&
         ReadData(planing_config.lattice_stop_buffer,"24.lattice_stop_buffer",root)&&
         ReadData(planing_config.enable_auto_tuning,"25.enable_auto_tuning",root)&&
         ReadData(planing_config.decision_horizon,"26.decision_horizon",root)&&
         ReadData(planing_config.trajectory_space_resolution,"27.trajectory_space_resolution",root)&&
         ReadData(planing_config.weight_lon_objective,"28.weight_lon_objective",root)&&
         ReadData(planing_config.weight_lon_jerk,"29.weight_lon_jerk",root)&&
         ReadData(planing_config.weight_lon_collision,"30.weight_lon_collision",root)&&
         ReadData(planing_config.weight_centripetal_acceleration,
                             "31.weight_centripetal_acceleration",root)&&
         ReadData(planing_config.weight_lat_offset,"32.weight_lat_offset",root)&&
         ReadData(planing_config.weight_lat_comfort,"33.weight_lat_comfort",root)&&
         ReadData(planing_config.lat_offset_bound,"34.lat_offset_bound",root)&&
         ReadData(planing_config.weight_opposite_side_offset,"35.weight_opposite_side_offset",root)&&
         ReadData(planing_config.weight_same_side_offset,"36.weight_same_side_offset",root)&&
         ReadData(planing_config.longitudinal_jerk_upper_bound,"37.longitudinal_jerk_upper_bound",root)&&
         ReadData(planing_config.longitudinal_jerk_lower_bound,"38.longitudinal_jerk_lower_bound",root)&&
         ReadData(planing_config.lateral_jerk_bound,"39.lateral_jerk_bound",root)&&
         ReadData(planing_config.weight_target_speed,"40.weight_target_speed",root)&&
         ReadData(planing_config.weight_dist_travelled,"41.weight_dist_travelled",root)&&
         ReadData(planing_config.lon_collision_cost_std,"42.lon_collision_cost_std",root)&&
         ReadData(planing_config.lon_collision_yield_buffer,"43.lon_collision_yield_buffer",root)&&
         ReadData(planing_config.lon_collision_overtake_buffer,"44.lon_collision_overtake_buffer",root)&&
         ReadData(planing_config.comfort_acceleration_factor,"45.comfort_acceleration_factor",root)&&
         ReadData(planing_config.lon_collision_buffer,"46.lon_collision_buffer",root)&&
         ReadData(planing_config.lat_collision_buffer,"47.lat_collision_buffer",root)&&
         ReadData(planing_config.kappa_bound,"48.kappa_bound",root)&&
         ReadData(planing_config.enable_trajectory_stitcher,"49.enable_trajectory_stitcher",root)&&
         ReadData(planing_config.replan_lateral_distance_threshold,
                             "50.replan_lateral_distance_threshold",root)&&
         ReadData(planing_config.replan_longitudinal_distance_threshold,
                             "51.replan_longitudinal_distance_threshold",root)&&
         ReadData(planing_config.enable_lag_prediction,"52.enable_lag_prediction",root)&&
         ReadData(planing_config.lag_prediction_min_appear_num,"53.lag_prediction_min_appear_num",root)&&
         ReadData(planing_config.lag_prediction_max_disappear_num,
                             "54.lag_prediction_max_disappear_num",root)&&
         ReadData(planing_config.look_forward_time_sec,"55.look_forward_time_sec",root)&&
         ReadData(planing_config.look_forward_short_distance,"56.look_forward_short_distance",root)&&
         ReadData(planing_config.look_forward_long_distance,"57.look_forward_long_distance",root)&&
         ReadData(planing_config.look_backward_distance,"58.look_backward_distance",root)&&
         ReadData(planing_config.enable_change_lane_decider,"59.enable_change_lane_decider",root)&&
         ReadData(planing_config.virtual_stop_wall_length,"60.virtual_stop_wall_length",root)&&
         ReadData(planing_config.align_prediction_time,"61.align_prediction_time",root)&&
         ReadData(planing_config.max_collision_distance,"62.max_collision_distance",root)&&
         ReadData(planing_config.ignore_overlapped_obstacle,"63.ignore_overlapped_obstacle",root)&&
         ReadData(planing_config.enable_collision_detection,"64.enable_collision_detection",root)&&
         ReadData(planing_config.prediction_message_history_limit,
                             "65.prediction_message_history_limit",root)&&
         ReadData(planing_config.perception_confidence_threshold,
                             "66.perception_confidence_threshold",root)&&
         ReadData(planing_config.lag_prediction_protection_distance,
                             "67.lag_prediction_protection_distance",root)&&
         ReadData(planing_config.cost_non_priority_reference_line,
                             "68.cost_non_priority_reference_line",root)&&
         ReadData(planing_config.planning_upper_speed_limit,"69.planning_upper_speed_limit",root)&&
         ReadData(planing_config.enable_backup_trajectory,"70.enable_backup_trajectory",root)&&
         ReadData(planing_config.backup_trajectory_cost,"71.backup_trajectory_cost",root)&&
         ReadData(planing_config.use_planning_fallback,"72.use_planning_fallback",root)&&
         ReadData(planing_config.planning_test_mode,"73.planning_test_mode",root)&&
         ReadData(planing_config.estimate_current_vehicle_state,"74.estimate_current_vehicle_state",root)&&
         ReadData(planing_config.planning_loop_rate,"75.planning_loop_rate",root)&&
         ReadData(planing_config.enable_record_debug,"76.enable_record_debug",root)&&
         ReadData(planing_config.publish_estop,"77.publish_estop",root)&&
         ReadData(planing_config.trajectory_time_high_density_period,
                             "78.trajectory_time_high_density_period",root)&&
         ReadData(planing_config.enable_stitch_last_trajectory,"79.enable_stitch_last_trajectory",root)&&
         ReadData(planing_config.enable_map_reference_unify,"80.enable_map_reference_unify",root)&&
         ReadData(planing_config.max_stop_distance_obstacle,"81.max_stop_distance_obstacle",root)&&
         ReadData(planing_config.min_stop_distance_obstacle,"82.min_stop_distance_obstacle",root)&&
         ReadData(planing_config.st_max_t,"83.st_max_t",root)&&
         ReadData(planing_config.st_max_s,"84.st_max_s",root)&&
         ReadData(planing_config.max_stop_speed,"85.max_stop_speed",root)&&
         ReadData(planing_config.default_cruise_speed,"86.default_cruise_speed",root)&&
         ReadData(planing_config.use_multi_thread_to_add_obstacles,
                             "87.use_multi_thread_to_add_obstacles",root)&&
         ReadData(planing_config.destination_check_distance,"88.destination_check_distance",root)&&
         ReadData(planing_config.destination_obstacle_id,"89.destination_obstacle_id",root)&&
         ReadData(planing_config.reckless_change_lane,"90.reckless_change_lane",root)&&
         ReadData(planing_config.change_lane_success_freeze_time,
                             "91.change_lane_success_freeze_time",root)&&
         ReadData(planing_config.change_lane_fail_freeze_time,"92.change_lane_fail_freeze_time",root)&&
         ReadData(planing_config.segment_length,"93.segment_length",root)&&
         ReadData(planing_config.interval_length,"94.interval_length",root)&&
         ReadData(planing_config.discretion_length,"95.discretion_length",root)&&
         ReadData(planing_config.clean_max_dis,"96.clean_max_dis",root)&&
         ReadData(planing_config.clean_max_width,"97.clean_max_width",root)&&
         ReadData(planing_config.clean_min_width,"98.clean_min_width",root)&&
         ReadData(planing_config.clean_max_evaluate_width,"99.clean_max_evaluate_width",root)&&
         ReadData(planing_config.clean_ignore_dis,"100.clean_ignore_dis",root)&&
         ReadData(planing_config.clean_effect_dis,"101.clean_effect_dis",root)&&
         ReadData(planing_config.ultra_front_safe_dis,"102.ultra_front_safe_dis",root)&&
         ReadData(planing_config.laterl_front_safe_dis,"103.laterl_front_safe_dis",root)&&
         ReadData(planing_config.laterl_back_safe_dis,"104.laterl_back_safe_dis",root)&&
         ReadData(planing_config.ultra_back_safe_dis,"105.ultra_back_safe_dis",root)&&
         ReadData(planing_config.weight_clean_garbage,
                            "106.weight_clean_garbage",root)&&
         ReadData(planing_config.planning_make_prediction_max_points_num,
                            "107.planning_make_prediction_max_points_num",root)&&
         ReadData(planing_config.gps_status_max_still_time,
                            "108.gps_status_max_still_time",root)&&
         ReadData(planing_config.ultrasonic_find_danger_min_still_time,
                            "109.ultrasonic_find_danger_min_still_time",root)&&
         ReadData(planing_config.enable_ploygon_checking,
                            "110.enable_ploygon_checking",root)&&
         ReadData(planing_config.enable_data_record,
                            "111.enable_data_record",root)&&
         ReadData(planing_config.enable_data_playback,
                            "112.enable_data_playback",root)&&
         ReadData(planing_config.slide_trans_dis,
                            "113.slide_trans_dis",root)&&
         ReadData(planing_config.prediction_point_num,
                            "114.prediction_point_num",root)&&
         ReadData(planing_config.sample_width,
                            "115.sample_width",root)&&
         ReadData(planing_config.max_angle_velocity,
                            "116.max_angle_velocity",root)&&
         ReadData(planing_config.max_beyond_side_buffer,
                            "117.max_beyond_side_buffer",root)&&
         ReadData(planing_config.max_slide_gap,
                            "118.max_slide_gap",root)&&
         ReadData(planing_config.cost_non_slide_reference_line,
                            "119.cost_non_slide_reference_line",root)&&
         ReadData(planing_config.default_slide_center_to_right_dis,
                            "120.default_slide_center_to_right_dis",root)&&
         ReadData(planing_config.obstacle_filter_buffer,
                            "121.obstacle_filter_buffer",root)&&
         ReadData(planing_config.ultra_stop_wait_time,
                            "122.ultra_stop_wait_time",root)&&
         ReadData(planing_config.decider_sample_dis,
                            "123.decider_sample_dis",root)&&
         ReadData(planing_config.combine_stop_result_dis,
                            "124.combine_stop_result_dis",root)&&
         ReadData(planing_config.valid_stop_decider_dis,
                            "125.valid_stop_decider_dis",root)&&
         ReadData(planing_config.valid_passing_obs_dis,
                            "126.valid_passing_obs_dis",root)&&
         ReadData(planing_config.per_error_dis,
                            "127.per_error_dis",root)




       ){
           cout << "Reading Complete!" << endl;
           in.close();
           return true;
       }else
        {
          cout << "ERROR Reading Incomplete !!!" << endl;
          in.close();
          return false;

        }

    } else {
      cout << "parse error\n" << endl;
       in.close();
       return false;
    }


}


bool ReadFile(string flie_path,VehicleParam & vehicle_param)
{

  Json::Reader reader;
  Json::Value root;
  //从文件中读取，保证当前文件有test.json文件
  ifstream in(flie_path,ios::binary);

  if(!in.is_open()){

     cout << "Error opening file\n";

     return false; }

  if(reader.parse(in,root)){
      //读取子节点信息
     if(
         ReadData(vehicle_param.front_edge_to_center,"01.front_edge_to_center",root)&&
         ReadData(vehicle_param.back_edge_to_center, "02.back_edge_to_center",root)&&
         ReadData(vehicle_param.left_edge_to_center, "03.left_edge_to_center",root)&&
         ReadData(vehicle_param.right_edge_to_center,"04.right_edge_to_center",root)&&
         ReadData(vehicle_param.length,"05.length",root)&&
         ReadData(vehicle_param.width, "06.width",root)&&
         ReadData(vehicle_param.height,"07.height",root)&&
         ReadData(vehicle_param.min_turn_radius, "08.min_turn_radius",root)&&
         ReadData(vehicle_param.max_acceleration,"09.max_acceleration",root)&&
         ReadData(vehicle_param.max_deceleration,"10.max_deceleration",root)&&
         ReadData(vehicle_param.max_steer_angle, "11.max_steer_angle",root)&&
         ReadData(vehicle_param.max_steer_angle_rate,"12.max_steer_angle_rate",root)&&
         ReadData(vehicle_param.min_steer_angle_rate,"13.min_steer_angle_rate",root)&&
         ReadData(vehicle_param.steer_ratio,"14.steer_ratio",root)&&
         ReadData(vehicle_param.wheel_base, "15.wheel_base",root)&&
         ReadData(vehicle_param.wheel_rolling_radius,"16.wheel_rolling_radius",root)&&
         ReadData(vehicle_param.max_abs_speed_when_stopped,
                            "17.max_abs_speed_when_stopped",root)

       ){
           cout << "Reading Complete!" << endl;
           in.close();
           return true;
        }else{
          cout << "ERROR Reading Incomplete !!!" << endl;
          in.close();
          return false;
        }
    } else {
      cout << "parse error\n" << endl;
       in.close();
       return false;
    }
}

bool ReadData(string &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asString();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}

bool ReadData(bool &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asBool();
      if(data)
         cout<<name<<" = "<<"ture"<<endl;
      else
         cout<<name<<" = "<<"false"<<endl;

      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}
bool ReadData(int &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asInt();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}
bool ReadData(uint32_t &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asUInt();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}
bool ReadData(double &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asDouble();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}

bool ConfigParamInit(string config_path,ConfigParam & planing_config )
{
  return ReadFile(config_path,planing_config);

}
bool VehicleParamInit(string vehicle_param_path,VehicleParam & vehicle_param )
{
  return ReadFile(vehicle_param_path,vehicle_param);

}

bool ReadTrajectory(std::vector<TrajectoryPoint>& trajectory_points){
  return ReadTrajectory(trajectory_flie_path,trajectory_points);
}

bool ReadObstacles(prediction::PredictionObstacles & prediction_obstacles){
  return ReadObstacles(obstacles_flie_path,prediction_obstacles);
}

} //end namespace planning
