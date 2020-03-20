
#include "planning.h"
#include "PlanPublish.h"
#include "common/serialize_data.h"
#include "common/get_now_time.h"
#include <memory.h>


extern planning::ConfigParam g_config_param;

namespace planning {



void Save_Plan_Input(
        const MSFLoutputS *msfl_output,
        const PredictionObstacles *obstcales,
        const Chassis *chassis,
        const UltrasonicSense *ultrasonic,
        const TrafficLightDetection *traffic_light,
        const hdmap::PncRoutes *pnc_routes)
{
    static double start_time = Clock::NowInSeconds();
    string file_name = "./log/data/plan/plan_input_" + Clock::GetYMDHMS(false)+".txt" ;
    static ofstream ofs(file_name, ios::out | ios::binary);

    if(Clock::NowInSeconds() - start_time > 600){
        start_time = Clock::NowInSeconds();
        file_name = "./log/data/plan/plan_input_" + Clock::GetYMDHMS(false)+".txt" ;
        ofs.close();
        ofs.open(file_name, ios::out | ios::binary);
    }

    if(ofs.is_open()){
        cout << "file open ok!!!" << endl;
    } else {
        cout << "file open failed!!!" << endl;
    }

    boost::archive::binary_oarchive oa(ofs);
    oa << *msfl_output;
    oa << *obstcales;
    oa << *chassis;
    oa << *ultrasonic;
    oa << *traffic_light;
    oa << *pnc_routes;
}

//*********************************************************


bool MFSL2LocationEstimate(const MSFLoutputS  *msfl_output_s_ptr,
                                 LocalizationEstimate  *localization_ptr){

  for( auto msfl : *msfl_output_s_ptr ) {

      Pose pose;
      pose.has_data = true;
      pose.gps_week = msfl.gpsWeek;
      pose.gps_week_sec = msfl.gpsSec;
      pose.timestamp_ms = msfl.timestamp;

      pose.point_llh.lon    = msfl.lon / 180.0 * M_PI;
      pose.point_llh.lat    = msfl.lat / 180.0 * M_PI;
      pose.point_llh.height = msfl.height;

      pose.euler_angles.x = msfl.pitch / 180.0 * M_PI;
      pose.euler_angles.y = msfl.roll  / 180.0 * M_PI;
      pose.euler_angles.z = msfl.yaw   / 180.0 * M_PI;

      pose.linear_velocity.x = msfl.velEast;
      pose.linear_velocity.y = msfl.velNorth;
      pose.linear_velocity.z = msfl.velUp;

      pose.linear_acceleration_vrf.x = msfl.accX;
      pose.linear_acceleration_vrf.y = msfl.accY;
      pose.linear_acceleration_vrf.y = msfl.accY;

      pose.angular_velocity_vrf.x = msfl.gyroX / 180.0 * M_PI;
      pose.angular_velocity_vrf.y = msfl.gyroY / 180.0 * M_PI;
      pose.angular_velocity_vrf.z = msfl.gyroZ / 180.0 * M_PI;

      switch (msfl.gpsState) {
        case 0:
          pose.gps_status = SINGAL_POINT;
          break;
        case 1:
          pose.gps_status = RTDGPS;
          break;
        case 2:
          pose.gps_status = SBAS;
          break;
        case 3:
          pose.gps_status = PPP;
          break;
        case 4:
          pose.gps_status = RTK_FLOAT;
          break;
        case 5:
          pose.gps_status = RTK_FIXATION;
          break;
        default:
          pose.gps_status = OTHER;
          break;
        }
      int nav = int(msfl.navState);
      //cout<<"msfl.navState = "<<nav<<endl;
      if(nav > 1){
          pose.is_nav  = true;
      }

      localization_ptr->pose_30.push_back(pose);
    }

  localization_ptr->data_size = localization_ptr->pose_30.size();
//  cout<<"(lon,lat) = "<<setprecision(12)     <<msfl_output_s_ptr->back().lon
//                      <<setprecision(12)<<","<<msfl_output_s_ptr->back().lat<<endl;
  if(localization_ptr->data_size > 0 )
    return true;
  else
    return false;

 }

PublishData::PbTrajectory PbTrajectory2Output(const PbTrajectory &pb_tra) {

  PublishData::PbTrajectory  pb_trajectory;
  pb_trajectory.e_stop                       = pb_tra.e_stop;
  pb_trajectory.map_orignal_point_llh.lat    = pb_tra.map_orignal_point_llh.lat ;
  pb_trajectory.map_orignal_point_llh.lon    = pb_tra.map_orignal_point_llh.lon ;
  pb_trajectory.map_orignal_point_llh.height = pb_tra.map_orignal_point_llh.height;
  pb_trajectory.pb_header             = pb_tra.pb_header;
  pb_trajectory.sweeper_signal        = pb_tra.sweeper_signal;
  pb_trajectory.trajectory_points     = pb_tra.trajectory_points;
  for(int i = 0 ; i < pb_trajectory.trajectory_points.size(); i++){
      pb_trajectory.trajectory_points.at(i).path_point.theta *= 180.0/M_PI;
    }

  pb_trajectory.trajectory_points_num = pb_tra.trajectory_points_num;
  pb_trajectory.vechicle_state        = pb_tra.vechicle_state;
  pb_trajectory.vehicle_signal        = pb_tra.vehicle_signal;

  return pb_trajectory;

 }


Planning g_planning;

bool PlanningInit(std::string param_config_path,
                  std::string vehicle_config_path)
{

  if(g_planning.Init(param_config_path,vehicle_config_path) != common::Status::OK){
      cout<<"Error planning init failed, please checking config param file "
            "and path or map file and path !"<<endl;
      return false;
    }

  cout<<"Info planning init successful, could call runonce function !"<<endl;
  return true;

}

std::string  PlanningVersion()  { return g_planning.Version();}


PublishData::PbTrajectory  PlanningRunOnceNew(
                        PredictionObstacles   *obstacles_ptr,
                        MSFLoutputS *msfl_output_s_ptr,
                        Chassis               *chassis_ptr,
                        TrafficLightDetection *traffic_detection_ptr,
                        CleanTarget           *clean_target_ptr,
                        UltrasonicSense       *ultrasonic_ptr,
                        SoftCommand           *command_ptr,
                        hdmap::PncRoutes      *pnc_routes_ptr)
{

  if( g_config_param.enable_data_record &&
     !g_config_param.enable_record_debug && !pnc_routes_ptr->empty())
    Save_Plan_Input(
                      msfl_output_s_ptr,
                      obstacles_ptr,
                      chassis_ptr,
                      ultrasonic_ptr,
                      traffic_detection_ptr,
                      pnc_routes_ptr );



  LocalizationEstimate  localization;
  if(!MFSL2LocationEstimate(msfl_output_s_ptr,&localization))
    cout<<"Error localization data is empty !"<<endl;



  return PbTrajectory2Output(
          g_planning.RunOnce( obstacles_ptr,  &localization,
                              chassis_ptr,     traffic_detection_ptr,
                              clean_target_ptr,ultrasonic_ptr,
                              command_ptr,     pnc_routes_ptr));
}

void PlanningStop()  { g_planning.Stop();}


} //namespace planning
