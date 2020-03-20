
#include "planning.h"

namespace planning {

PbTrajectory Planning::RunOnce(PredictionObstacles   *obstacles_ptr,
                               LocalizationEstimate  *localization_ptr,
                               Chassis               *chassis_ptr,
                               TrafficLightDetection *traffic_detection_ptr,
                               CleanTarget           *clean_target_ptr,
                               UltrasonicSense       *ultrasonic_,
                               SoftCommand           *command_ptr,
                               hdmap::PncRoutes      *pnc_routes_ptr) {

     return planning_base_->RunOnce(obstacles_ptr,
                                    localization_ptr,
                                    chassis_ptr  ,
                                    traffic_detection_ptr,
                                    clean_target_ptr,
                                    ultrasonic_ ,
                                    command_ptr,
                                    pnc_routes_ptr);
 }


} //namespace planning


