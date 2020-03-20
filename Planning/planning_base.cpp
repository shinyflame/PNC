/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "planning_base.h"
#include <algorithm>
#include <list>
#include <vector>
#include "math/quaternion.h"
#include "common/get_now_time.h"
#include "common/trajectory/trajectory_stitcher.h"
#include "planner/lattice/lattice_planner.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

namespace planning {
using common::Status;
PlanningBase::~PlanningBase() {}

bool PlanningBase::IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x) || std::isnan(vehicle_state.y) ||
      std::isnan(vehicle_state.z) || std::isnan(vehicle_state.heading) ||
      std::isnan(vehicle_state.kappa) ||
      std::isnan(vehicle_state.linear_velocity) ||
      std::isnan(vehicle_state.linear_acceleration)) {

    cout<<"Error vehicle state is invalid !!! "<<endl;
    return false;

  }
  return true;
}

PbTrajectory PlanningBase::PublishPlanningPb(PbTrajectory* trajectory_pb,uint64_t timestamp)
{
  trajectory_pb->pb_header.plan_timestamp = timestamp;//ms
  // TODO(all): integrate reverse gear
  //trajectory_pb->gear = GEAR_DRIVE;
  if (g_config_param.use_planning_fallback &&
      trajectory_pb->trajectory_points.size() == 0) {
    cout<<"Fall back Trajectory at planning base !!!"<<endl;
    SetFallbackTrajectory(trajectory_pb);
  }
  // NOTICE:
  // Since we are using the time at each cycle beginning as timestamp, the
  // relative time of each trajectory point should be modified so that we can
  // use the current timestamp in header.
  if (!g_config_param.planning_test_mode)
  {
    int i = 0;
    for(auto& p : trajectory_pb->trajectory_points)
      { 
        p.relative_time = i * g_config_param.trajectory_time_resolution;
        i++;
      }
  }
  double now_time_hours = Clock::NowInHours();
  if(now_time_hours >17 || now_time_hours < 8){
      trajectory_pb->vehicle_signal.high_beam = true;
      trajectory_pb->vehicle_signal.low_beam = true;

  }else{
      trajectory_pb->vehicle_signal.high_beam = false;
      trajectory_pb->vehicle_signal.low_beam = false;
  }

  trajectory_pb->vehicle_signal.emergency_light = false;
  trajectory_pb->sweeper_signal.fan_Hz = 150;
  trajectory_pb->sweeper_signal.front_brush =  MOVE_DOWN;
  trajectory_pb->sweeper_signal.front_brush_speed = 10;
  trajectory_pb->trajectory_points_num =
  trajectory_pb->trajectory_points.size();
  if(trajectory_pb->trajectory_points.empty()){
     cout<<"Warning trajectory is empty"<<endl;
     trajectory_pb->e_stop = true;
     trajectory_pb->vehicle_signal.emergency_light = true;
     trajectory_pb->sweeper_signal.fan_Hz = 80;
  }

  KeepWithinBoundForAngleVelocity(trajectory_pb);
  cout<<"plan end   timestamp = "<<Clock::NowInMs()<<endl;
  cout<<"now time is: "<<Clock::GetYMDHMS(true)<<endl;
  int num = 0;
//  for(auto tra_point : trajectory_pb->trajectory_points){
//      if(num >6 ) break;
//      cout<<"t = "<<tra_point.relative_time<<endl;
//      cout<<"v = "<<tra_point.v<<endl;
//      cout<<"a = "<<tra_point.a<<endl;
//      cout<<"s = "<<tra_point.path_point.s<<endl;
//      cout<<"x = "<<tra_point.path_point.x<<endl;
//      cout<<"y = "<<tra_point.path_point.y<<endl;
//      cout<<"theta = "<<tra_point.path_point.theta<<endl;
//      cout<<"kappa = "<<tra_point.path_point.kappa<<endl<<endl;
//      num++;

//    }

  return *trajectory_pb;
}

void PlanningBase::KeepWithinBoundForAngleVelocity(PbTrajectory* trajectory_pb){
    double max_w = g_config_param.max_angle_velocity / 180.0 * M_PI;
    for(int i = 0 ;i < trajectory_pb->trajectory_points_num; i++){
        auto v = trajectory_pb->trajectory_points.at(i).v;
        auto k = trajectory_pb->trajectory_points.at(i).path_point.kappa;
        double w = abs(v * k);
        if(w > max_w){
            trajectory_pb->trajectory_points.at(i).v = abs(max_w /k);
//            cout<<"tra point i = "<<i<<"w beyond max_w, so reset v, original v = "
//                <<v<<", but changed later v = "
//                <<trajectory_pb->trajectory_points.at(i).v<<endl;
        }
        if( trajectory_pb->trajectory_points.at(i).v >
            g_config_param.default_cruise_speed){
            trajectory_pb->trajectory_points.at(i).v =
            g_config_param.default_cruise_speed;

        }
        if(i > 0){
          trajectory_pb->trajectory_points.at(i).a =
            ( trajectory_pb->trajectory_points.at(i).v -
              trajectory_pb->trajectory_points.at(i-1).v)/
             g_config_param.trajectory_time_resolution;
        }



    }
}

ADCTrajectory PlanningBase::Publish(ADCTrajectory* trajectory) {
  //AdapterManager::UndataLatestTrajectory(trajectory);
  return *trajectory;
}
void PlanningBase::SetFallbackTrajectory(PbTrajectory* trajectory_pb) {

  // use planning trajecotry from last cycle
  if (AdapterManager::GetPlanning())
  {
    const auto& traj = AdapterManager::GetLatestTrajectory();
    const uint64_t current_time_stamp = trajectory_pb->pb_header.plan_timestamp;
    const uint64_t pre_time_stamp = traj.pb_header.plan_timestamp;
    for (int i = 0; i < traj.trajectory_points.size(); ++i)
    {
      const double t = traj.trajectory_points[i].relative_time +
      (current_time_stamp - pre_time_stamp)/1000.0 * (-1);//updata relatively time
      TrajectoryPoint point;
      point = traj.trajectory_points[i];
      point.relative_time = t;
      trajectory_pb->trajectory_points.push_back(point);
    }
    AdapterManager::ClearLatestTrajectory ();
  }
}

}  // namespace planning

