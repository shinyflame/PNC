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

#ifndef MODULES_PLANNING_STD_PLANNING_H_
#define MODULES_PLANNING_STD_PLANNING_H_

#include <memory>
#include <string>
#include <vector>

//#include "modules/common/util/thread_pool.h"
#include "common/frame.h"
//#include "planner/std_planner_dispatcher.h"
#include "planning_base.h"

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
 * @brief Planning module main class. It processes GPS and IMU as input,
 * to generate planning info.
 */
class StdPlanning : public PlanningBase {
 public:
  StdPlanning() = default;
  virtual ~StdPlanning();

  /**
   * @brief Planning algorithm name.
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  common::Status Init(std::string param_config_path,
                      std::string vehicle_config_path) override;

  /**
   * @brief module start function
   * @return start status
   */
  common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief main logic of the planning module, runs periodically triggered by
   * timer.
   */
  PbTrajectory RunOnce( PredictionObstacles   *obstacles_ptr,
                        LocalizationEstimate  *localization_ptr,
                        Chassis               *chassis_ptr,
                        TrafficLightDetection *traffic_detection_ptr,
                        CleanTarget           *clean_target_ptr,
                        UltrasonicSense       *ultrasonic_,
                        SoftCommand           *command_ptr,
                        hdmap::PncRoutes             *pnc_routes_ptr) override;

  void OnTimer() override;

  bool GetTrafficeRule(TrafficRuleStruct::TrafficRuleConfigs &Rulers);

  common::Status Plan(
      const uint64_t current_time_stamp,
      const std::vector<TrajectoryPoint>& stitching_trajectory,
      PbTrajectory* trajectory) override;
  bool UltrasonicResultIsDanger(UltrasonicSense *ultrasonic_,bool disable_right_back);

 private:
  common::Status InitFrame(const uint32_t sequence_num,
                           const TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const VehicleState& vehicle_state ,
                           const vector<Garbage> &Garbages);
  //void ExportReferenceLineDebug(planning_internal::Debug* debug);
  //bool CheckPlanningConfig();

 private:
  //routing::RoutingResponse last_routing_;
  std::unique_ptr<Frame> frame_;
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;
  double gps_status_lower_start_time_;
  double ultrasonic_find_danger_start_time_;
  bool first_run_ = true;
  hdmap::MapPoint map_orignal_point_;
  uint64_t run_times_ = 0;
  bool init_status_ = false;
  bool ultrasonic_find_danger_ = false;
  double danger_stop_begin_time_;
  TurnSignal turn_light_signal_;


};

}  // namespace planning


#endif /* MODULES_PLANNING_STD_PLANNING_H_ */
