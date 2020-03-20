
#pragma once

#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/basic_binary_iprimitive.hpp>
#include <string> 
#include <sstream>

#include <fstream>
#include <utility>

//#include "MSFLPublish.h"
//#include "PlanPublish.h"
#include "struct.h"
#include "../map/map_struct.h"


namespace boost {
namespace serialization {
/*---------------------------------------------------------INPUT----------------------------------*/
/*-------------------MSFLoutput--------------------*/
template<class Archive>
void serialize(Archive & ar, MSFLoutput& msfl_output, const unsigned int version)
{
    ar & msfl_output.head;
    ar & msfl_output.timestamp;
    ar & msfl_output.gpsWeek;
    ar & msfl_output.gpsSec;
    ar & msfl_output.lat;
    ar & msfl_output.lon;
    ar & msfl_output.height;
    ar & msfl_output.velEast;
    ar & msfl_output.velNorth;
    ar & msfl_output.velUp;
    ar & msfl_output.pitch;
    ar & msfl_output.roll;
    ar & msfl_output.yaw;

    ar & msfl_output.sigmaLat;
    ar & msfl_output.sigmaLon;
    ar & msfl_output.sigmaHeight;
    ar & msfl_output.sigmaVelEast;
    ar & msfl_output.sigmaVelNorth;
    ar & msfl_output.sigmaVelUp;
    ar & msfl_output.sigmaPitch;
    ar & msfl_output.sigmaRoll;
    ar & msfl_output.sigmaYaw;

    ar & msfl_output.accX;
    ar & msfl_output.accY;
    ar & msfl_output.accZ;
    ar & msfl_output.gyroX;
    ar & msfl_output.gyroY;
    ar & msfl_output.gyroZ;

    ar & msfl_output.navState;
    ar & msfl_output.gpsState;

    ar & msfl_output.newNavOutFlag;

    ar & msfl_output.hardwareFaultInfo;
    ar & msfl_output.inputParamFaultInfo;
    ar & msfl_output.alignFaultInfo;
    ar & msfl_output.sensorOutLostInfo;
    ar & msfl_output.alignState;
    ar & msfl_output.fault2;
}

/*-------------------PERCEPTION---------------------*/


template<class Archive>
void serialize(Archive & ar, planning::perception::Point  & polygon_point,
               const unsigned int version)
{
    ar & polygon_point.x;
    ar & polygon_point.y;
    ar & polygon_point.z;
}

template<class Archive>
void serialize(Archive & ar, planning::perception::Velocity  & velocity,
               const unsigned int version)
{
    ar & velocity.x;
    ar & velocity.y;
    ar & velocity.z;
}
template<class Archive>
void serialize(Archive & ar,
               planning::perception::PerceptionObstacle  & perception_obstacle,
               const unsigned int version)
{
    ar & perception_obstacle.confidence;
    ar & perception_obstacle.height;
    ar & perception_obstacle.id;
    ar & perception_obstacle.length;
    ar & perception_obstacle.polygon_num;
    ar & perception_obstacle.polygon_point;
    ar & perception_obstacle.position;
    ar & perception_obstacle.theta;
    ar & perception_obstacle.tracking_time;
    ar & perception_obstacle.type;
    ar & perception_obstacle.velocity;
    ar & perception_obstacle.width;
}
template<class Archive>
void serialize(Archive & ar,
               planning::prediction::Trajectory  & trajectory,
               const unsigned int version)
{
    ar & trajectory.probability;
    ar & trajectory.trajectory_point;
    ar & trajectory.trajectory_point_num;
}

template<class Archive>
void serialize(Archive & ar,
               planning::prediction::PredictionObstacle  & prediction_obstacle,
               const unsigned int version)
{
    ar & prediction_obstacle.perception_obstacle;
    ar & prediction_obstacle.predicted_period;
    ar & prediction_obstacle.timestamp;
    ar & prediction_obstacle.trajectory;
    ar & prediction_obstacle.trajectory_num;
}

template<class Archive>
void serialize(Archive & ar,
               planning::prediction::PredictionObstacles& prediction_obstacles,
               const unsigned int version)
{

    ar & prediction_obstacles.prediction_obstacle_num;
    ar & prediction_obstacles.prediction_obstacle;
    ar & prediction_obstacles.start_timestamp;
    ar & prediction_obstacles.end_timestamp;
}

/*-------------------CHASSIS------------------------*/

template<class Archive>
void serialize(Archive & ar, planning::VehicleSignal & signal,
               const unsigned int version)
{
    ar & signal.emergency_light;
    ar & signal.high_beam;
    ar & signal.horn;
    ar & signal.low_beam;
    ar & signal.turn_signal;
}

template<class Archive>
void serialize(Archive & ar,
               planning::canbus::Chassis & chassis,
               const unsigned int version)
{

    ar & chassis.battery_range_km;//1
    ar & chassis.brake_percentage;//2
    ar & chassis.driving_mode;    //3
    ar & chassis.engine_rpm;      //4
    ar & chassis.engine_started;  //5
    ar & chassis.gear_location;   //6
    ar & chassis.has_data;        //7
    ar & chassis.odometer_m;      //8
    ar & chassis.parking_brake;   //9
    ar & chassis.signal;          //10
    ar & chassis.speed_mps;       //11
    ar & chassis.splice_angle;    //12
    ar & chassis.steering_percentage;//13
    ar & chassis.steering_torque_nm; //14
    ar & chassis.throttle_percentage;//15
    ar & chassis.timestamp_ms;       //16
    ar & chassis.wiper;              //17
}


/*-------------------ULTRASONIC SENSE---------------*/
template<class Archive>
void serialize(Archive & ar,
               planning::UltrasonicSense & ultrasonic_sense,
               const unsigned int version)
{
    ar & ultrasonic_sense.time_stamp;
    ar & ultrasonic_sense.front;
    ar & ultrasonic_sense.back;
    ar & ultrasonic_sense.left;
    ar & ultrasonic_sense.right;
}

template<class Archive>
void serialize(Archive & ar, planning::Ultrasonic & front, const unsigned int version)
{
    ar & front.distance;
    ar & front.id;
    ar & front.valid;
}

/*-------------------TRAFFIC LIGHT------------------*/
template<class Archive>
void serialize(Archive & ar,
               planning::perception::TrafficLight & traffic_light,
               const unsigned int version)
{
    ar & traffic_light.color;
    ar & traffic_light.confidence;
    ar & traffic_light.id;
    ar & traffic_light.tracking_time;
}
template<class Archive>
void serialize(Archive & ar,
               planning::perception::TrafficLightDetection & traffic_light_detection,
               const unsigned int version)
{
    ar & traffic_light_detection.camera_timestamp;
    ar & traffic_light_detection.frame_num;
    ar & traffic_light_detection.traffic_light;

}
/*-------------------MAP----------------------------*/

template<class Archive>
void serialize(Archive & ar,
               hdmap::EulerAngles & euler_angles, const unsigned int version)
{
    ar & euler_angles.pitch;
    ar & euler_angles.roll;
    ar & euler_angles.yaw;
}

template<class Archive>
void serialize(Archive & ar,
               hdmap::Point3D & point_enu, const unsigned int version)
{
    ar & point_enu.x;
    ar & point_enu.y;
    ar & point_enu.z;
}

template<class Archive>
void serialize(Archive & ar,
               hdmap::PointLLH & point_llh, const unsigned int version)
{
    ar & point_llh.height;
    ar & point_llh.lat;
    ar & point_llh.lon;
}



template<class Archive>
void serialize(Archive & ar,
               hdmap::Terminal& terminal, const unsigned int version){

    ar & terminal.end_s;
    ar & terminal.id;
    ar & terminal.start_s;

}

template<class Archive>
void serialize(Archive & ar,
               hdmap::Station& station, const unsigned int version){

    ar& station.end_s;
    ar& station.id;
    ar& station.start_s;
    ar& station.stop_duration_allowed;

}
template<class Archive>
void serialize(Archive & ar,
               hdmap::TrafficLight& traffic_light, const unsigned int version){
    ar& traffic_light.end_s;
    ar& traffic_light.has_traffic_light;
    ar& traffic_light.ids;
    ar& traffic_light.start_s;

}

template<class Archive>
void serialize(Archive & ar,
               hdmap::Width& widths, const unsigned int version){

    ar& widths.center_to_left;
    ar& widths.center_to_right;
    ar& widths.s;

}

template<class Archive>
void serialize(Archive & ar,
               hdmap::MapPoint & map_original_point, const unsigned int version)
{
    ar & map_original_point.euler_angles;
    ar & map_original_point.kappa;
    ar & map_original_point.point_enu;
    ar & map_original_point.point_llh;
    ar & map_original_point.s;
}

template<class Archive>
void serialize(Archive & ar,
               hdmap::Route & route, const unsigned int version){

    ar& route.id;
    ar& route.lanes_id;
    ar& route.reference_points;
    ar& route.widths;
    ar& route.traffic_light;
    ar& route.speed_limit;
    ar& route.turn_type;
    ar& route.station;
    ar& route.terminal;


}



template<class Archive>
void serialize(Archive & ar, hdmap::PncRoute& pnc_route, const unsigned int version)
{
    ar& pnc_route.side_slip;
    ar& pnc_route.is_update;
    ar& pnc_route.on_route;
    ar& pnc_route.map_original_point;
    ar& pnc_route.route;
}

/*-------------------PLAN OUTPUT--------------------*/
#if 1
template<class Archive>
void serialize(Archive & ar, planning::PbHeader& pb_header, const unsigned int version)
{
    ar & pb_header.plan_timestamp;
    ar & pb_header.prediction_timestamp;
    ar & pb_header.camera_timestamp;
    ar & pb_header.chassis_timestamp;
    ar & pb_header.location_timestamp;
}

//template<class Archive>
//void serialize(Archive & ar, PublishData::PointLLH& point_llh, const unsigned int version)
//{
//    ar & point_llh.lon ;
//    ar & point_llh.lat ;
//    ar & point_llh.height ;
//}

//template<class Archive>
//void serialize(Archive & ar, PublishData::Point3D& point_3d, const unsigned int version)
//{
//    ar & point_3d.x ;
//    ar & point_3d.y ;
//    ar & point_3d.z ;
//}

template<class Archive>
void serialize(Archive & ar, planning::VehiclePositonState& vehicle_state, const unsigned int version)
{
    ar & vehicle_state.point_llh;
    ar & vehicle_state.velocity_xyz;
    ar & vehicle_state.acceleration_xyz;
    ar & vehicle_state.euler_angles;
    ar & vehicle_state.angular_velocity;
}

template<class Archive>
void serialize(Archive & ar, planning::PathPoint& path_point, const unsigned int version)
{
    ar & path_point.x;
    ar & path_point.y;
    ar & path_point.z;
    ar & path_point.s;
    ar & path_point.theta;
    ar & path_point.kappa;
    ar & path_point.dkappa;
    ar & path_point.ddkappa;
}

template<class Archive>
void serialize(Archive & ar, planning::TrajectoryPoint& trajectory_point,
               const unsigned int version)
{
    ar & trajectory_point.path_point;
    ar & trajectory_point.v ;
    ar & trajectory_point.a ;
    ar & trajectory_point.relative_time;
}

#if 0
template<class Archive>
void serialize(Archive & ar, planning::VehicleSignal& vehicle_signal,
               const unsigned int version)
{
    ar & vehicle_signal.turn_signal;
    ar & vehicle_signal.high_beam;
    ar & vehicle_signal.low_beam;
    ar & vehicle_signal.horn;
    ar & vehicle_signal.emergency_light;
}
#endif

template<class Archive>
void serialize(Archive & ar, planning::SweeperSignal& sweeper_signal, const unsigned int version)
{
    ar & sweeper_signal.front_brush;
    ar & sweeper_signal.front_brush_speed;
    ar & sweeper_signal.behind_brush;
    ar & sweeper_signal.behind_brush_speed;
    ar & sweeper_signal.fan_Hz;
    ar & sweeper_signal.water_rate;
    ar & sweeper_signal.shaker_state;
    ar & sweeper_signal.container_state;
}

template<class Archive>
void serialize(Archive & ar, planning::PbTrajectory& pb_trajectory, const unsigned int version)
{
    ar & pb_trajectory.pb_header;
    ar & pb_trajectory.map_orignal_point_llh;
    ar & pb_trajectory.vechicle_state;
    ar & pb_trajectory.e_stop;
    ar & pb_trajectory.trajectory_points_num;
    ar & pb_trajectory.trajectory_points;
    ar & pb_trajectory.vehicle_signal;
    ar & pb_trajectory.sweeper_signal;
}

#endif



} // namespace serialization
} // namespace boost









