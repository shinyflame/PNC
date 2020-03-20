

#ifndef GET_DATA_H
#define GET_DATA_H

#include <functional>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include "../common/struct.h"
#include "../common/coordinate_convert.h"
#include "../map/map_struct.h"

namespace planning{


namespace AdapterManager {

  using perception::PerceptionObstacle;
  using perception::TrafficLightDetection;
  using prediction::PredictionObstacle;
  using prediction::PredictionObstacles;
  using localization::LocalizationEstimate;
  using PlanningStatusStruct::PlanningStatus;
  using canbus::Chassis;



  bool Initialized(
                    const PredictionObstacles   *obstacles_ptr,
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
                          hdmap::PncRoutes      &pnc_routes  );


  bool  GetPrediction() ;
  PredictionObstacles *GetLatestObserved();
  std::list<PredictionObstacles> *GetPredictionLists() ;

  bool  GetLocalization();
  void  UpdataLocalization(LocalizationEstimate latest_localiztion);
  const LocalizationEstimate &GetLatestLocalization();
  bool GpsTimeToUnixTime(LocalizationEstimate * localization_ptr);


  bool GetPlanning();
  PlanningStatus* GetPlanningStatus();
  PbTrajectory GetLatestTrajectory();
  void UndataLatestTrajectory(PbTrajectory *trajectory);
  void ClearLatestTrajectory ();


  bool GetRoutingResponse();

  bool  GetChassis();
  const Chassis& GetLatestChassisObserved() ;

  bool   GetTrafficLightDetection();
  double GetTrafficLightDetectionDelaySec();
  TrafficLightDetection GetTrafficLightDetectionLatestObserved();
  Pose FindMatchLocation(uint64_t obs_timestamp,
                         LocalizationEstimate  &localization_,
                         const hdmap::PointLLH &map_original_llh);
  void MakePredictionForObstacles(PredictionObstacles   &obstacles_,
                                  const Pose &match_pose);
  CleanTarget   GetCleanTarget();

} //namespace AdapterManager

}//namespace planning

#endif // GET_DATA_H
