
#include "std_planning.h"
#include <algorithm>
#include <list>
#include <memory>
#include <utility>
#include <vector>
#include "math/quaternion.h"
#include "common/ego_info.h"
#include "common/trajectory/trajectory_stitcher.h"
#include "planner/lattice/lattice_planner.h"
#include "reference_line/reference_line_provider.h"
#include "toolkits/deciders/traffic_decider.h"
#include "common/get_data.h"
#include "common/get_now_time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/get_traffic_rule.h"
#include "config/planning_config_param_initial.h"




//using namespace hdmap;

planning::ConfigParam g_config_param;
planning::VehicleParam g_vehicle_config;

namespace planning {



using common::Status;

StdPlanning::~StdPlanning() { /*Stop();*/ }

std::string StdPlanning::Name() const { return "V2.2.6"; }

common::Status StdPlanning::Init(std::string param_config_path,
                                 std::string vehicle_config_path) {

  if(!ConfigParamInit(param_config_path,g_config_param)){
      cout <<"Config Param Init failed !!!"<<endl;
      return Status::PLANNING_ERROR;
    }
  if(!VehicleParamInit(vehicle_config_path,g_vehicle_config)){
      cout <<"Vehicle Param Init failed!!!"<<endl;
      return Status::PLANNING_ERROR;
    }

  if(!GetTrafficeRule(traffic_rule_configs_)) {
      cout << "Failed to load traffic rule "<<endl;
      return Status::PLANNING_ERROR;
    }

  planner_.reset(new LatticePlanner());
  if (planner_ == nullptr) {
    cout<<"Error planner is nullptr...in std_planning.cpp on 234 line"<<endl;
    return Status::PLANNING_ERROR ;
  }
  init_status_ = true;
  return planner_->Init();
}

Status StdPlanning::InitFrame(const uint32_t sequence_num,
                              const TrajectoryPoint& planning_start_point,
                              const double start_time,
                              const VehicleState& vehicle_state,
                              const vector<Garbage> &Garbages) {
  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,
                         vehicle_state, reference_line_provider_.get(),Garbages));
  auto status = frame_->Init();
  if (status != Status::OK ) {
    cout << "failed to init frame:" << endl;
    return status;
  }
  return Status::OK;
}


PbTrajectory StdPlanning::RunOnce( PredictionObstacles   *obstacles_ptr,
                                   LocalizationEstimate  *localization_ptr,
                                   Chassis               *chassis_ptr,
                                   TrafficLightDetection *traffic_detection_ptr,
                                   CleanTarget           *clean_target_ptr,
                                   UltrasonicSense       *ultrasonic_,
                                   SoftCommand           *command_ptr,
                                   hdmap::PncRoutes      *pnc_routes_ptr
                                 )
{

  PbTrajectory pb_trajectory;
  const uint64_t start_timestamp = Clock::NowInMs();
  cout<<"planning run times = "<<run_times_++<<endl;

  PredictionObstacles   obstacles_;
  LocalizationEstimate  localization_;
  Chassis               chassis_;
  TrafficLightDetection traffic_detection_;
  CleanTarget           clean_target_;
  hdmap::PncRoutes      pnc_routes_;
  bool initial_successful =
  AdapterManager::Initialized(
                              obstacles_ptr,         obstacles_   ,
                              localization_ptr,      localization_,
                              chassis_ptr,           chassis_ ,
                              traffic_detection_ptr, traffic_detection_,
                              clean_target_ptr,      clean_target_ ,
                              pnc_routes_ptr,        pnc_routes_ );

  if(!initial_successful){
      cout<<"ERROR initial failed ....!"<<endl;
      return PublishPlanningPb(&pb_trajectory, start_timestamp);
    }

  map_orignal_point_ = pnc_routes_.front().map_original_point;
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(pnc_routes_ptr);

  pb_trajectory.pb_header.location_timestamp   = localization_.pose_30.back().timestamp_ms;
  pb_trajectory.pb_header.prediction_timestamp = obstacles_.start_timestamp;
  pb_trajectory.pb_header.chassis_timestamp    = chassis_.timestamp_ms;
  pb_trajectory.pb_header.camera_timestamp     = traffic_detection_.camera_timestamp;
  pb_trajectory.map_orignal_point_llh.lat    = map_orignal_point_.point_llh.lat;
  pb_trajectory.map_orignal_point_llh.lon    = map_orignal_point_.point_llh.lon;

  std::unique_ptr<VehicleStateProvider> vehicle_state_provider(new VehicleStateProvider());
  Status status = vehicle_state_provider->Update(localization_, chassis_);
  VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  pb_trajectory.vechicle_state = vehicle_state_provider->position_state();

  if (status != Status::OK || !IsVehicleStateValid(vehicle_state)) {
    cout << "Update VehicleStateProvider failed"<<endl;
    return PublishPlanningPb(&pb_trajectory, start_timestamp);
  }

  if(first_run_){
      gps_status_lower_start_time_ = Clock::NowInSeconds();
      ultrasonic_find_danger_start_time_ = Clock::NowInSeconds();
      if(localization_.pose_30.back().gps_status == RTK_FIXATION &&
         localization_.pose_30.back().is_nav == true ) {
         cout<<"msfl data is valid begin planning ..."<<endl;
         first_run_ = false;
      }else{
         cout<<"Error msfl data is invalid waiting msfl init complete ..."<<endl;
         return PublishPlanningPb(&pb_trajectory, start_timestamp);
      }

   }

  //gps status lower deal
  auto gps_status = localization_.pose_30.back().gps_status;
  if(gps_status == RTK_FIXATION || gps_status == RTK_FLOAT ){
      gps_status_lower_start_time_ = Clock::NowInSeconds();
   }
  double gps_status_lower_still_time =
                    Clock::NowInSeconds() - gps_status_lower_start_time_;
  if(gps_status_lower_still_time > g_config_param.gps_status_max_still_time){

      cout<<"GPS status lower timeover Planning failed ....!"<<endl;
      return PublishPlanningPb(&pb_trajectory, start_timestamp);
    }


  // estimate (x, y) at current timestamp
  // This estimate is only valid if the current time and vehicle state timestamp
  // differs only a small amount (20ms). When the different is too large, the
  // estimation is invalid.
  //DCHECK_GE(start_timestamp, vehicle_state.timestamp());
  if (g_config_param.estimate_current_vehicle_state &&
      start_timestamp - vehicle_state.timestamp_ms < 20) {
    auto future_xy = vehicle_state_provider->EstimateFuturePosition(
        (start_timestamp - vehicle_state.timestamp_ms)/1000.0);
    vehicle_state.x = future_xy.x();
    vehicle_state.y = future_xy.y();
    vehicle_state.timestamp_ms = start_timestamp;
  }

  reference_line_provider_->UpdateVehicleState(vehicle_state);
  const double planning_cycle_time = 1.0 / g_config_param.planning_loop_rate;

  std::vector<TrajectoryPoint> stitching_trajectory;
  stitching_trajectory = TrajectoryStitcher::
  ComputeStitchingTrajectory( vehicle_state,reference_line_provider_.get()->GetAdcSLPoint(),start_timestamp,
                              planning_cycle_time,last_publishable_trajectory_.get());

  static uint32_t  sequence_num=0; sequence_num++;
  const  uint32_t frame_num = sequence_num ;
  status =
  InitFrame( frame_num, stitching_trajectory.back(),
             start_timestamp,vehicle_state, clean_target_.Garbages);
  if (status != Status::OK ) {
    if (g_config_param.publish_estop) {
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      cout<<"Frame initial failed"<<endl;
      pb_trajectory.e_stop = true;
      return PublishPlanningPb(&pb_trajectory, start_timestamp);
    } else {
      cout<<"Frame initial failed"<<endl;
      return PublishPlanningPb(&pb_trajectory, start_timestamp);
    }
  }

  if (!frame_) {
    cout << "Failed to init frame" <<endl;
    return PublishPlanningPb(&pb_trajectory, start_timestamp);
  }



#if 0
  //ultrasonic danger deal
  if(!UltrasonicResultIsDanger(ultrasonic_,frame_.get()->NoUsingRightBackUltra())){
      ultrasonic_find_danger_start_time_ = Clock::NowInSeconds();
    }
  double ultrasonic_find_danger_still_time =
      Clock::NowInSeconds() - ultrasonic_find_danger_start_time_;
  if( ultrasonic_find_danger_still_time >
      g_config_param.ultrasonic_find_danger_min_still_time ) {
    cout<<"Note Ultrasonice danger still time is = "
        << ultrasonic_find_danger_still_time<<endl;
    cout<<"Ultrasonic checked potential danger skip the planning cycle !!!"<<endl;
    ultrasonic_find_danger_ = true;
    danger_stop_begin_time_ = Clock::NowInSeconds();
    return PublishPlanningPb(&pb_trajectory, start_timestamp);
  }
  //danger disappear waiting time
  if( ultrasonic_find_danger_ &&
      Clock::NowInSeconds() - danger_stop_begin_time_ <
      g_config_param.ultra_stop_wait_time) {
      return PublishPlanningPb(&pb_trajectory, start_timestamp);
    } else {
      ultrasonic_find_danger_ = false;
    }

#endif

  frame_->SetCommand(*command_ptr);

  EgoInfo::instance()->Update( stitching_trajectory.back(), vehicle_state,
                               frame_->obstacles() );

  if(g_config_param.planning_test_mode){
      cout<<"Note now plannig mode is control tune test !"<<endl;
      for (auto& ref_line_info : frame_->reference_line_info()) {

          auto& reference_line  = ref_line_info.reference_line();
          auto reference_line_points = reference_line.reference_points();
          double vehicle_s = ref_line_info.GetVechicleSLPoint().s;
          int vehicle_match_index = reference_line.GetReferenceMatchPointIndex(vehicle_s);

          for(int i = 0 ; i < 81 ; i++ ) {

              TrajectoryPoint tra_p;
              if(vehicle_match_index + i < 0 ||
                 vehicle_match_index + i >= reference_line_points.size()){
                  cout<<"Error current index beyond reference line points max size !"<<endl;
                  return PublishPlanningPb(&pb_trajectory, start_timestamp);
               } else {
                  tra_p.a = 0;
                  tra_p.v = 1.0;
                  tra_p.relative_time = i * g_config_param.trajectory_time_resolution;
                  tra_p.path_point = reference_line_points.at(vehicle_match_index + i);
                  tra_p.path_point.s -= reference_line_points.at(vehicle_match_index).s;
               }
              pb_trajectory.trajectory_points.push_back(tra_p);
           }
          return PublishPlanningPb(&pb_trajectory, start_timestamp);
        }
    }

  for (auto& ref_line_info : frame_->reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
    if (traffic_status != Status::OK || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      cout << "Reference line "        << ref_line_info.Lanes().Id()
           << " traffic decider failed"<<endl;
      continue;
    }
  }


  status = Plan(start_timestamp, stitching_trajectory, &pb_trajectory);
  const auto time_diff_ms = Clock::NowInMs() - start_timestamp ;
  cout << "total planning time spend: " << time_diff_ms << " ms......................8"<<endl;

  if (status != Status::OK)
   {
    cout << "Planning failed:" << status<<endl;
    if (g_config_param.publish_estop)
     {
      cout << "Planning failed and set estop" <<endl;
      // Because the function "Control::ProduceControlCommand()" checks the
      // "estop" signal with the following line (Line 170 in control.cc):
      // estop_ = estop_ || trajectory_.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      pb_trajectory.e_stop = true;
    }
  }

  cout << "Planning pb:" << status<<endl<<endl;
  cout <<"vehicle_lon   =" <<pb_trajectory.vechicle_state.point_llh.lon<<endl;
  cout <<"vehicle_lat   =" <<pb_trajectory.vechicle_state.point_llh.lat<<endl;
  cout <<"vehicle_hei   =" <<pb_trajectory.vechicle_state.point_llh.height<<endl<<endl;

  cout <<"vehicle_x   =" <<vehicle_state.x<<endl;
  cout <<"vehicle_y   =" <<vehicle_state.y<<endl;
  cout <<"vehicle_x   =" <<vehicle_state.z<<endl<<endl;

  cout <<"vehicle_rool  =" <<vehicle_state.roll<<endl;
  cout <<"vehicle_pitch =" <<vehicle_state.pitch<<endl;
  cout <<"vehicle_yaw   =" <<vehicle_state.yaw<<endl<<endl;

  cout <<"vehicle_linear_velocity    = " <<vehicle_state.linear_velocity<<endl;
  cout <<"vehicle_inear_acceleration = " <<vehicle_state.linear_acceleration<<endl;
  cout <<"vehicle_angular_velocity   = " <<vehicle_state.angular_velocity<<endl;
  cout <<"vehicle_kappa              = " <<vehicle_state.kappa<<endl<<endl;
  pb_trajectory.map_orignal_point_llh.height = vehicle_state.linear_velocity;
  pb_trajectory.vehicle_signal.turn_signal = turn_light_signal_;
  cout <<"plan start_timestamp = "<<start_timestamp<<endl;

  return PublishPlanningPb(&pb_trajectory, start_timestamp);
}


Status StdPlanning::Plan( const uint64_t current_time_stamp,
                          const std::vector<TrajectoryPoint>& stitching_trajectory,
                          PbTrajectory* trajectory_pb)
{
  //1.start lattice planing
  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get());

  //2.获取最佳古迹所对的参考线信息
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (!best_ref_info ||
       best_ref_info->trajectory().NumOfPoints() == 0) {//如果没有得到则返回，规划失败

     cout<<"planner failed to make a driving plan"<<endl;
//    if (last_publishable_trajectory_) {//清空上一次保存的轨迹
//      last_publishable_trajectory_->Clear();
//    }
    return Status::PLANNING_ERROR ;
  }
  //3.if get best ref info 保存此次规划的轨迹，下次规划使用
  last_publishable_trajectory_.reset(new PublishableTrajectory(
   current_time_stamp, best_ref_info->trajectory()));
  cout << "current_time_stamp: " << current_time_stamp <<endl;

  //4.如果开启缝合标志位，则将缝合轨迹加入到轨迹中
  if (g_config_param.enable_stitch_last_trajectory /*&&
      stitching_trajectory.end()->v>0.50*/) {
       last_publishable_trajectory_->PrependTrajectoryPoints(
       stitching_trajectory.begin(), stitching_trajectory.end());
  }

  //5.将轨迹填充到 trajectory_pb
  last_publishable_trajectory_->PopulateTrajectoryProtobuf(trajectory_pb);

  //6.save trajectory to
  AdapterManager::UndataLatestTrajectory(trajectory_pb);
  //7. get turn light type
  const auto & route_segment = best_ref_info->Lanes();
  auto turn_type = route_segment.GetRoute().turn_type;
  if(turn_type == LEFT_TURN){
      turn_light_signal_ = TURN_LEFT;
  }else if(turn_type == RIGHT_TURN){
      turn_light_signal_ = TURN_RIGHT;
  }else{
      turn_light_signal_ = TURN_NONE;
  }

  if(!best_ref_info->Lanes().IsOnSegment() &&
      best_ref_info->trajectory().NumOfPoints() > 0){
      if(best_ref_info->IsSideSlipClean()){
          turn_light_signal_ = TURN_RIGHT;
      }else{
          turn_light_signal_ = TURN_LEFT;
      }
  }

  return status;
}

bool StdPlanning::GetTrafficeRule(TrafficRuleConfigs &Rulers)
{
    return TrafficRuleStruct::GetTrafficeRule(Rulers);
}

void StdPlanning::Stop() {
  cout << "Planning Stop is called"<<endl;
  reference_line_provider_->Stop();
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  planner_.reset(nullptr);
  EgoInfo::instance()->Clear();
}

Status StdPlanning::Start() {

  if(!reference_line_provider_->Start())
    {
      cout<<"reference_line_provider_ Error"<<endl;
      return Status::PLANNING_ERROR;
    }
  return Status::OK;
}

void StdPlanning::OnTimer() {

    //RunOnce();
}
/**
* @brief StdPlanning::UltrasonicResult if checked danger return true else return false
* @param ultrasonic_
* @return bool
*
*/
bool StdPlanning::UltrasonicResultIsDanger(UltrasonicSense *ultrasonic_,
                                           bool disable_right_back){

  double front_safe_dis = g_config_param.ultra_front_safe_dis;//2.0;
  double laterl_front_safe_dis = g_config_param.laterl_front_safe_dis; //0.5;
  double laterl_back_safe_dis =  g_config_param.laterl_back_safe_dis;//0.5;
  double  back_safe_dis = g_config_param.ultra_back_safe_dis; //0.4;
//  double time_gap_upper = 0.8; //second
//  auto time_gap = (Clock::NowInMs() - ultrasonic_->time_stamp)/1000;
//  if(time_gap > time_gap_upper){

//      cout<<"time_gap = "<< time_gap <<"beyond upper 0.8s Ultrasonic data is invalid"<<endl;
//      return false;
//  }
  for(auto sensor_data : ultrasonic_->front ){
      if(sensor_data.valid && sensor_data.distance < front_safe_dis)
        {
          cout<<"ultrasonic data = "<<sensor_data.distance
              <<" < safety value :"<<front_safe_dis<<endl;
          cout<<"Front Ultrasonic sensor id:"<<sensor_data.id<<" checked danger !!!"<<endl;
          return true;
        }
    }
  for(auto sensor_data : ultrasonic_->back ){
      if(sensor_data.valid && sensor_data.distance < back_safe_dis)
        {
          cout<<"ultrasonic data = "<<sensor_data.distance
              <<" < safety value :" <<back_safe_dis <<endl;
          cout<<"Back Ultrasonic sensor id:"<<sensor_data.id<<" checked danger !!!"<<endl;
          return true;
        }
    }


   if((ultrasonic_->left.front().valid &&
       ultrasonic_->left.front().distance <  laterl_front_safe_dis) ||
      (ultrasonic_->right.front().valid &&
       ultrasonic_->right.front().distance < laterl_front_safe_dis)
     ){
       cout<<"Left front ultrasonic data = "<<ultrasonic_->left.front().distance<<endl;
       cout<<"Left front ultrasonic sensor id:"<<ultrasonic_->left.front().id<<endl;
       cout<<"Right front ultrasonic data = "<<ultrasonic_->right.front().distance<<endl;
       cout<<"Left  front ultrasonic sensor id:"<<ultrasonic_->right.front().id<<endl;
       cout<<"Laterl_front_safe_dis :" <<laterl_front_safe_dis <<endl;

       return true;
     }

   if( ultrasonic_->left.back().valid &&
       ultrasonic_->left.back().distance <  laterl_back_safe_dis    ){
       cout<<"Left back ultrasonic data = "<<ultrasonic_->left.back().distance<<endl;
       cout<<"Left back ultrasonic sensor id:"<<ultrasonic_->left.back().id<<endl;
       cout<<"Laterl_back_safe_dis :" <<laterl_back_safe_dis <<endl;

       return true;
    }


   if((!disable_right_back) && ultrasonic_->right.back().valid &&
      ultrasonic_->right.back().distance < laterl_back_safe_dis){
       cout<<"Right back ultrasonic data = "<<ultrasonic_->right.back().distance<<endl;
       cout<<"Right back ultrasonic sensor id:"<<ultrasonic_->right.back().id<<endl;
       cout<<"Laterl_back_safe_dis :" <<laterl_back_safe_dis <<endl;

       return true;
    }





  return false;
}

}  // namespace planning

