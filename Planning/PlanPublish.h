

#ifndef PLAN_PUBLISH_H_
#define PLAN_PUBLISH_H_

#include <vector>
#include <string>
#include "common/struct.h"
#include "map/map_struct.h"
//#include "MSFLPublish.h"

namespace planning {

using perception::TrafficLightDetection;
using prediction::PredictionObstacles;
using localization::LocalizationEstimate;
using canbus::Chassis;
//using namespace hdmap;


  extern "C" std::string  PlanningVersion();

  extern "C" bool PlanningInit( std::string param_config_path,
                                std::string vehicle_config_path) ;

  extern "C" PublishData::PbTrajectory  PlanningRunOnceNew(
                        PredictionObstacles   *obstacles_ptr,
                        MSFLoutputS           *msfl_output_s_ptr,
                        Chassis               *chassis_ptr,
                        TrafficLightDetection *traffic_detection_ptr,
                        CleanTarget           *clean_target_ptr,
                        UltrasonicSense       *ultrasonic_,
                        SoftCommand           *command_ptr,
                        hdmap::PncRoutes      *pnc_routes_ptr   );



  extern "C" void PlanningStop() ;



}//end namespace
#endif // PLAN_H
