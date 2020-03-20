

#ifndef COORDINATE_CONVERT_H
#define COORDINATE_CONVERT_H

#include "../common/struct.h"
#include "../map/map_struct.h"
namespace planning {


class CoordinateConvert
{
public:
  //Coordinate_convert() = delete;
  static bool LastToCurrent(const VehiclePositonState &last_vehicle_state,
                     const VehiclePositonState &current_vehicle_state,
                     std::vector<TrajectoryPoint> &stitching_trajectory);

  static bool LlhToMapXyz(const PointLLH &current_LLH,
                   const Point3D &current_euler_angles,
                   const hdmap::PointLLH &map_original_LLH,
                   const hdmap::Point3D &map_original_euler_angles,
                   Point3D &position);
  static bool PointRelativeTranslationAndRotaion(Point3D &InputPoint,
                             const Point3D &TransValue,
                             const double &RotValue);
  static bool TranslationAndRotaion(PathPoint &InputPoint,
                             const Point3D &TransValue,
                             const double &RotValue);
  static bool TranslationAndRotaion(Point3D &InputPoint,
                             const Point3D &TransValue,
                             const double &RotValue);
  static bool TranslationAndRotaion(perception::Point &InputPoint,
                             const Point3D &TransValue,
                             const double &RotValue);
  static bool TranslationAndRotaion(perception::Point &InputPoint,
                              const perception::Point &TransValue,
                              const double &RotValue);
  static bool TranslationAndRotaion(PathPoint &InputPoint,
                             const  perception::Point &TransValue,
                             const  double &RotValue);

  static bool FromCarToMapPosition(prediction::PredictionObstacles &obstacles_,
                            const Point3D &TransValue,
                            const double &RotValue  );
  static bool FromCarToMapPosition(CleanTarget   &clean_target_ ,
                                   const Point3D &TransValue,
                                   const double  &RotValue  );
  static bool LlhToMapXyz( Pose &current_LLH,
                           const hdmap::PointLLH &map_original_LLH );

};


} //end namespace planning


#endif // COORDINATE_CONVERT_H
