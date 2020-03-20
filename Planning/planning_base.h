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

#ifndef MODULES_PLANNING_PLANNING_BASE_H_
#define MODULES_PLANNING_PLANNING_BASE_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

//#include "ctpl/ctpl_stl.h"
//include "modules/planning/proto/traffic_rule_config.pb.h"
//#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "common/trajectory/publishable_trajectory.h"
#include "planner/planner.h"
#include "common/struct.h"
#include "map/map_struct.h"
/**
 * @namespace planning
 * @brief planning
 */

namespace planning {

using perception::PerceptionObstacle;
using perception::TrafficLightDetection;
using prediction::PredictionObstacle;
using prediction::PredictionObstacles;
using localization::LocalizationEstimate;
using PlanningStatusStruct::PlanningStatus;
using canbus::Chassis;



/**
 * @class planning
 *
 * @brief PlanningBase module main class.
 */
class PlanningBase  {
 public:
  PlanningBase() = default;
  virtual ~PlanningBase();

  virtual PbTrajectory RunOnce( PredictionObstacles   *obstacles_ptr,
                                LocalizationEstimate  *localization_ptr,
                                Chassis               *chassis_ptr,
                                TrafficLightDetection *traffic_detection_ptr,
                                CleanTarget           *clean_target_ptr,
                                UltrasonicSense       *ultrasonic_,
                                SoftCommand           *command_ptr,
                                hdmap::PncRoutes      *pnc_routes_ptr) = 0;

  virtual std::string Name() const = 0;

  virtual common::Status Init(std::string param_config_path,
                              std::string vehicle_config_path) = 0;

  virtual common::Status Start() = 0;

  virtual void Stop() = 0;

  // Watch dog timer
  virtual void OnTimer() = 0;

  /**
   * @brief Plan the trajectory given current vehicle state
   */
  virtual common::Status Plan(
      const uint64_t current_time_stamp,
      const std::vector<TrajectoryPoint>& stitching_trajectory,
      PbTrajectory* trajectory) = 0;

 protected:

  /**
   * @brief Fill the header and publish the planning message.
   */
  PbTrajectory PublishPlanningPb(PbTrajectory* trajectory_pb, uint64_t timestamp);
  ADCTrajectory Publish(ADCTrajectory* trajectory) ;
  void KeepWithinBoundForAngleVelocity(PbTrajectory* trajectory_pb) ;
  bool IsVehicleStateValid(const VehicleState& vehicle_state);
  virtual void SetFallbackTrajectory(PbTrajectory* cruise_trajectory);
  //virtual bool CheckPlanningConfig() = 0;

 protected:
  uint64_t start_time_ = 0.0;

  TrafficRuleStruct::TrafficRuleConfigs traffic_rule_configs_;
  const hdmap::HDMap* hdmap_ = nullptr;
  std::unique_ptr<Planner> planner_;
  std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;

};

}  // namespace planning


#endif  // MODULES_PLANNING_PLANNING_BASE_H_
