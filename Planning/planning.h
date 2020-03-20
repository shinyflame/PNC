

#ifndef PLANNING_H_
#define PLANNING_H_

#include <memory>
#include <string>
#include "planning_base.h"
#include "std_planning.h"


namespace planning {

/**
 * @class planning
 *
 * @brief Planning module main class.
 */


class Planning  {

public:
  Planning() {  planning_base_.reset(new StdPlanning());}

  virtual ~Planning() = default;


  std::string  Version() const  { return planning_base_->Name(); }

  common::Status Init(std::string param_config_path,
                      std::string vehicle_config_path) {

    return planning_base_->Init(param_config_path,vehicle_config_path);
  }

  PbTrajectory RunOnce( PredictionObstacles   *obstacles_ptr,
                        LocalizationEstimate  *localization_ptr,
                        Chassis               *chassis_ptr,
                        TrafficLightDetection *traffic_detection_ptr,
                        CleanTarget           *clean_target_ptr,
                        UltrasonicSense       *ultrasonic_,
                        SoftCommand           *command_ptr,
                        hdmap::PncRoutes      *pnc_routes_ptr
                        );



  void Stop()  { return planning_base_->Stop(); }

private:
  std::unique_ptr<PlanningBase> planning_base_;
  LocalizationEstimate  localization_;

};


} // end namespace planning

#endif /* PLANNING_H_ */
