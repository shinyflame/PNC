
#include "../common/coordinate_convert.h"
#include <Eigen/Dense>


using namespace Eigen;


namespace planning {



bool CoordinateConvert::LastToCurrent(const VehiclePositonState &last_vehicle_state,
                                       const VehiclePositonState &current_vehicle_state,
                                      std::vector<TrajectoryPoint> &stitching_trajectory){

  const double a = 6378137;//meter  a half of earth  length aixs
  const double f = 1.0 / 298.257;
  const double e = sqrt(2.0 * f - f*f);

  double r0 = current_vehicle_state.point_llh.lon ;    //longitude
  double l0 = current_vehicle_state.point_llh.lat ;   //latitude
  double h0 = current_vehicle_state.point_llh.height; //height

  double r1 = last_vehicle_state.point_llh.lon ;
  double l1 = last_vehicle_state.point_llh.lat ;
  double h1 = last_vehicle_state.point_llh.height;

  double theta_alafa = current_vehicle_state.euler_angles.z -
                       last_vehicle_state.euler_angles.z   ;
  //cout<<"theta_alafa = "<<theta_alafa<<endl;
  double N0  = a/sqrt(1 - e*e*sin(l0)*sin(l0));
  double N1  = a/sqrt(1 - e*e*sin(l1)*sin(l1));
  Matrix<double, 3, 3> R , Lz;
  Vector3d S , T ,U;

  R << -sin(r0) ,          cos(r0) ,         0 ,
       -sin(l0)*cos(r0) , -sin(r0)*sin(l0),  cos(l0),
        cos(r0)*cos(l0),   cos(l0)*sin(r0),  sin(l0);

  S << (N1 + h1)*cos(l1)*cos(r1) - (N0+ h0)*cos(l0)*cos(r0),
       (N1 + h1)*cos(l1)*sin(r1) - (N0+ h0)*cos(l0)*sin(r0),
       (N1*(1-e*e) + h1)*sin(l1) - (N0*(1-e*e) + h0)*sin(l0);

  T = R * S ;

  Lz << cos(theta_alafa) , sin(theta_alafa) , 0 ,
       -sin(theta_alafa) , cos(theta_alafa) , 0 ,
         0               ,   0              , 1 ;

  for(int i = 0 ; i < stitching_trajectory.size() ; i ++){

      U << stitching_trajectory.at(i).path_point.x,
           stitching_trajectory.at(i).path_point.y,
           stitching_trajectory.at(i).path_point.z;

      U = Lz * (U + T);
//    U =  (U + T);
//    double x = U[0] * cos(theta_alafa) +  U[1]*sin(theta_alafa);
//    double y = U[1] * cos(theta_alafa) -  U[0]*sin(theta_alafa);

      stitching_trajectory.at(i).path_point.x = U[0];
      stitching_trajectory.at(i).path_point.y = U[1];
      stitching_trajectory.at(i).path_point.z = U[2];

    }

  return 1;

}

bool CoordinateConvert::LlhToMapXyz( const PointLLH &current_LLH,
                                     const Point3D  &current_euler_angles,
                                     const hdmap::PointLLH &map_original_LLH,
                                     const hdmap::Point3D  &map_original_euler_angles,
                                           Point3D  &position )
{

  const double a = 6378137;//meter  a half of earth  length aixs
  const double f = 1.0 / 298.257;
  const double e = sqrt(2.0 * f - f*f);

  double r0 = map_original_LLH.lon ;
  double l0 = map_original_LLH.lat ;
  double h0 = map_original_LLH.height;

  double r1 = current_LLH.lon ;    //longitude
  double l1 = current_LLH.lat ;   //latitude
  double h1 = current_LLH.height; //height

  double theta_alafa = map_original_euler_angles.z - current_euler_angles.z ;
  //cout<<"theta_alafa = "<<theta_alafa<<endl;
  double N  = a/sqrt(1 - e*e*sin(l0)*sin(l0));

  Matrix<double, 3, 3> R , Lz;
  Vector3d S , T ,U;

  R << -sin(r0) ,          cos(r0) ,         0 ,
       -sin(l0)*cos(r0) , -sin(r0)*sin(l0),  cos(l0),
        cos(r0)*cos(l0),   cos(l0)*sin(r0),  sin(l0);

  S << (N + h1)*cos(l1)*cos(r1) - (N+ h0)*cos(l0)*cos(r0),
       (N + h1)*cos(l1)*sin(r1) - (N+ h0)*cos(l0)*sin(r0),
       (N*(1-e*e) + h1)*sin(l1) - (N*(1-e*e) + h0)*sin(l0);

  T = R * S ;

  Lz << cos(theta_alafa) , sin(theta_alafa) , 0 ,
       -sin(theta_alafa) , cos(theta_alafa) , 0 ,
         0               ,   0              , 1 ;

  U << position.x,
       position.y,
       position.z;

  U = Lz * U ;

  position.x = U[0] + T[0];
  position.y = U[1] + T[1];
  position.z = U[2] + T[2];

  return 1;

}

bool CoordinateConvert::LlhToMapXyz( Pose  &pose_,
                                     const hdmap::PointLLH &map_original_LLH )
{

  const double a = 6378137;//meter  a half of earth  length aixs
  const double f = 1.0 / 298.257;
  const double e = sqrt(2.0 * f - f*f);
  auto current_LLH = pose_.point_llh;
  if((abs(map_original_LLH.lon - current_LLH.lon) > 0.01)||
     (abs(map_original_LLH.lat - current_LLH.lat) > 0.01)){
      cout<<"P_Error GPS (lat or lon) is too far away from map orignal point !!! "<<endl;
      cout<<"Location (lat,lon) = ("<<current_LLH.lon * 180.0/M_PI<<", "
                                    <<current_LLH.lat * 180.0/M_PI<<endl;
      cout<<"Original (lat,lon) = ("<<map_original_LLH.lon * 180.0/M_PI<<", "
                                    <<map_original_LLH.lat * 180.0/M_PI<<endl;

      return false;
    }


  double r0 = map_original_LLH.lon  ;
  double l0 = map_original_LLH.lat  ;
  double h0 = map_original_LLH.height;


  double r1 = current_LLH.lon ;    //longitude
  double l1 = current_LLH.lat ;    //latitude
  double h1 = current_LLH.height; //height

  //cout<<"theta_alafa = "<<theta_alafa<<endl;
  double N  = a/sqrt(1 - e*e*sin(l0)*sin(l0));

  Matrix<double, 3, 3> R ;
  Vector3d S , T ;

  R << -sin(r0) ,          cos(r0) ,         0 ,
       -sin(l0)*cos(r0) , -sin(r0)*sin(l0),  cos(l0),
        cos(r0)*cos(l0),   cos(l0)*sin(r0),  sin(l0);

  S << (N + h1)*cos(l1)*cos(r1) - (N+ h0)*cos(l0)*cos(r0),
       (N + h1)*cos(l1)*sin(r1) - (N+ h0)*cos(l0)*sin(r0),
       (N*(1-e*e) + h1)*sin(l1) - (N*(1-e*e) + h0)*sin(l0);

  T = R * S ;

  pose_.position.x = T[0];
  pose_.position.y = T[1];
  pose_.position.z = T[2];

  double yaw = pose_.euler_angles.z;
  if(yaw < M_PI){
      yaw *= -1;
   }else{

      yaw -= M_PI * 2.0 ;
      yaw *= -1;
   }
  pose_.euler_angles.z = yaw;

  cout<<"From LLH to map position convert successful !!!"<<endl;
  cout<<"Vehicle Position ( X, Y, Z ,yaw) = ("<<pose_.position.x<<", "<<pose_.position.y
      <<", "<<pose_.position.z<<", "<<pose_.euler_angles.z<<")"<<endl<<endl;
  return true;

}

bool CoordinateConvert::PointRelativeTranslationAndRotaion(
        Point3D &InputPoint,const Point3D &TransValue,const double &RotValue){

  Matrix<double, 3, 3>  Lz;
  Vector3d  U;

  Lz << cos(RotValue) , sin(RotValue) , 0 ,
       -sin(RotValue) , cos(RotValue) , 0 ,
         0            ,   0           , 1 ;
  U << InputPoint.x,
       InputPoint.y,
       InputPoint.z;

  U = Lz * U ;

  InputPoint.x = U[0] + TransValue.x;
  InputPoint.y = U[1] + TransValue.y;
  InputPoint.z = U[2] + TransValue.z;

  return 1;

}

bool CoordinateConvert::TranslationAndRotaion(
        PathPoint &InputPoint,const Point3D &TransValue,const double &RotValue){

  Matrix<double, 3, 3>  Lz;
  Vector3d  U;

  Lz << cos(RotValue) , sin(RotValue) , 0 ,
       -sin(RotValue) , cos(RotValue) , 0 ,
         0            ,   0           , 1 ;
  U << InputPoint.x,
       InputPoint.y,
       InputPoint.z;

  U = Lz * U ;

  InputPoint.x = U[0] + TransValue.x;
  InputPoint.y = U[1] + TransValue.y;
  InputPoint.z = U[2] + TransValue.z;

  return 1;

}

bool CoordinateConvert::TranslationAndRotaion(
     perception::Point &InputPoint,const Point3D &TransValue,const double &RotValue){

  Matrix<double, 3, 3>  Lz;
  Vector3d  T ,U;

  Lz << cos(RotValue) , sin(RotValue) , 0 ,
       -sin(RotValue) , cos(RotValue) , 0 ,
         0            ,   0           , 1 ;
  U << InputPoint.x,
       InputPoint.y,
       InputPoint.z;

  U = Lz * U ;

  InputPoint.x = U[0] + TransValue.x;
  InputPoint.y = U[1] + TransValue.y;
  InputPoint.z = U[2] + TransValue.z;

  return 1;

}

bool CoordinateConvert::TranslationAndRotaion(
     Point3D &InputPoint,const Point3D &TransValue,const double &RotValue){

  Matrix<double, 3, 3>  Lz;
  Vector3d  T ,U;

  Lz << cos(RotValue) , sin(RotValue) , 0 ,
       -sin(RotValue) , cos(RotValue) , 0 ,
         0            ,   0           , 1 ;
  U << InputPoint.x,
       InputPoint.y,
       InputPoint.z;

  U = Lz * U ;

  InputPoint.x = U[0] + TransValue.x;
  InputPoint.y = U[1] + TransValue.y;
  InputPoint.z = U[2] + TransValue.z;

  return 1;

}

bool CoordinateConvert::TranslationAndRotaion(perception::Point &InputPoint,
                           const  perception::Point &TransValue,
                           const  double &RotValue)
{

  Matrix<double, 3, 3>  Lz;
  Vector3d  T ,U;

  Lz << cos(RotValue) , sin(RotValue) , 0 ,
       -sin(RotValue) , cos(RotValue) , 0 ,
         0            ,   0           , 1 ;
  U << InputPoint.x,
       InputPoint.y,
       InputPoint.z;

  U = Lz * U ;

  InputPoint.x = U[0] + TransValue.x;
  InputPoint.y = U[1] + TransValue.y;
  InputPoint.z = U[2] + TransValue.z;

  return 1;

}

bool CoordinateConvert::TranslationAndRotaion(PathPoint &InputPoint,
                           const  perception::Point &TransValue,
                           const  double &RotValue)
{

  Matrix<double, 3, 3>  Lz;
  Vector3d  T ,U;

  Lz << cos(RotValue) , sin(RotValue) , 0 ,
       -sin(RotValue) , cos(RotValue) , 0 ,
         0            ,   0           , 1 ;
  U << InputPoint.x,
       InputPoint.y,
       InputPoint.z;

  U = Lz * U ;

  InputPoint.x = U[0] + TransValue.x;
  InputPoint.y = U[1] + TransValue.y;
  InputPoint.z = U[2] + TransValue.z;

  return 1;

}

bool CoordinateConvert::
FromCarToMapPosition(prediction::PredictionObstacles &obstacles_,
                     const Point3D &TransValue,const double &RotValue ) {

  for(int i = 0 ; i < obstacles_.prediction_obstacle.size() ; i++ )
    {
      TranslationAndRotaion(
        obstacles_.prediction_obstacle.at(i).perception_obstacle.position,
        TransValue,RotValue);
      obstacles_.prediction_obstacle.at(i).perception_obstacle.theta -= RotValue;

      for(int j = 0; j <
         obstacles_.prediction_obstacle.at(i).perception_obstacle.polygon_point.size();
          j++)
       {
         TranslationAndRotaion(
          obstacles_.prediction_obstacle.at(i).perception_obstacle.polygon_point.at(j)
          ,TransValue,RotValue);
       }

      for(int j = 0 ; j <obstacles_.prediction_obstacle.at(i).trajectory.size(); j++)
        {
         for(int k = 0 ;
             k < obstacles_.prediction_obstacle.at(i).trajectory.at(j).trajectory_point.size();
             k++)
           {
             TranslationAndRotaion(obstacles_.prediction_obstacle.at(i).trajectory.at(j).
                                   trajectory_point.at(k).path_point, TransValue, RotValue );
             double theta = obstacles_.prediction_obstacle.at(i).trajectory.at(j).
                            trajectory_point.at(k).path_point.theta - RotValue;
             if(theta > M_PI){
                 theta = theta - 2.0*M_PI;
             }else if(theta < -M_PI){
                 theta = theta + 2.0*M_PI;
             }

             obstacles_.prediction_obstacle.at(i).trajectory.at(j).
                                         trajectory_point.at(k).path_point.theta = theta;

           }

        }

    }

}


bool CoordinateConvert::FromCarToMapPosition(CleanTarget   &clean_target_ ,
                                             const Point3D &TransValue,
                                             const double  &RotValue  ){

  for(auto &garbage:clean_target_.Garbages){

      TranslationAndRotaion(garbage.position ,TransValue,RotValue);
      for(auto angle_point:garbage.polygons){
          TranslationAndRotaion(angle_point ,TransValue,RotValue);
        }
    }


}

} //end namespace planning
