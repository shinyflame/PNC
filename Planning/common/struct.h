#ifndef STRUCT_H
#define STRUCT_H
#include "string"
#include <vector>
#include <array>
#include <map>
#include <iostream>

//#pragma pack(1)
//#define uint32_t_t unsigned int
//#define int32_t int
//#define uint64_t unsigned long long
//#define int64 long long

using namespace std;

namespace planning {


using State = std::array<double, 3>;
using Condition = std::pair<State, double>;
using FilePath = std::array<string, 3>;



  struct PathTimePoint {
    double t ;
    double s ;
    int obstacle_id;
  };

  struct SamplePoint {
     PathTimePoint path_time_point ;
     double ref_v ;  };

  struct PathTimeObstacle { //ST graph 障碍物表达结构体
    int32_t       obstacle_id  ;
    PathTimePoint bottom_left  ;
    PathTimePoint upper_left   ;
    PathTimePoint upper_right  ;
    PathTimePoint bottom_right ;
    double time_lower ;
    double time_upper ;
    double path_lower ;
    double path_upper ;
  };

  enum StopType {
               HARD = 1,
               SOFT   };

  struct StopPoint {
          double s  ;
          StopType type ;};//default soft

  struct	PlanningTarget {
    StopPoint stop_point ;
    bool has_stop_point = false;//default false
    double cruise_speed ;
  } ;

  struct PointENU {
     double x ;        // East from the origin, in meters.
     double y ;        // North from the origin, in meters.
     double z =  0.0;  // Up from the WGS-84 ellipsoid, in meters.
  };

  // A point in the global reference frame. Similar to PointENU, PointLLH allows
  // omitting the height field for representing a 2D location.
  struct Point3D {
    double x ;
    double y ;
    double z ;
  };

  // A general 2D point. Its meaning and units depend on context, and must be
  // explained in comments.
  struct Point2D {
    double x ;
    double y ;
  };

  struct PointLLH {
  // Longitude in degrees, ranging from -180 to 180.
    double lon ;
  // Latitude in degrees, ranging from -90 to 90.
    double lat ;
  // WGS-84 ellipsoid height in meters.
    double height ;};

  // A unit quaternion that represents a spatial rotation. See the link below for
  // details.
  //   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  // The scalar part qw can be omitted. In this case, qw should be calculated by
  //   qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
  struct Quaternion {
    double qx ;
    double qy ;
    double qz ;
    double qw ;
    //bool has_data =false;
  };

  // A general polygon, points are counter clockwise
  struct Polygon {
    vector<Point3D> point;
  };
  enum GpsStatus{
    SINGAL_POINT = 0,
    RTDGPS,
    SBAS,
    PPP,
    RTK_FLOAT,
    RTK_FIXATION,
    OTHER
  };
  enum NavStatus{
    INIT = 0,
    INS,
    GPS,
    MC,
    ODE,
    LIDAR,
    CAMERA,

  };
  struct Pose {
    // Position of the vehicle reference point (VRP) in the map reference frame.
    // The VRP is the center of rear axle.

    int    gps_week;
    double gps_week_sec;
    uint64_t timestamp_ms ;
    PointLLH point_llh;    // 全球地理坐标系下的经纬高
    Point3D position ;//车辆后轴中心在地图中的坐标 m
    // Linear velocity of the VRP in the map reference frame.
    // East/north/up in meters per second.
    Point3D linear_velocity ;//在东北天坐标系中车辆的线速度m/s
    // Linear acceleration of the VRP in the vehicle reference frame.
    // Right/forward/up in meters per square second.
    Point3D linear_acceleration_vrf ;//车身坐标系 加速度m/s^2
    // Angular velocity of the VRP in the vehicle reference frame.
    // Around right/forward/up axes in radians per second.
    Point3D angular_velocity_vrf ;// 车身坐标系 角速度rad/s
    // Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.
    // in world coordinate (East/North/Up)
    // The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.
    // The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.
    // The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
    // The direction of rotation follows the right-hand rule.
    Point3D euler_angles ; //符合右手定则角度单位 rad
    GpsStatus gps_status;
    bool is_nav = false;
    bool has_data = false;
  };

  enum GearPosition {
      GEAR_NEUTRAL = 0,
      GEAR_DRIVE ,      //驾驶
      GEAR_REVERSE ,    //倒车
      GEAR_PARKING ,    //停车
      GEAR_LOW ,        // 低速
      GEAR_INVALID ,    //无效
      GEAR_NONE ,       //空挡
  };

  enum DrivingMode {
      COMPLETE_MANUAL = 0,     // human drive
      COMPLETE_AUTO_DRIVE ,    //自动
      AUTO_STEER_ONLY ,        // only steer
      AUTO_SPEED_ONLY ,        // include throttle and brake油门和刹车
      // security mode when manual intervention happens, only response status
      EMERGENCY_MODE
  };



  enum VehicleBrand {  LINCOLN_MKZ = 0,
                        GEM };

  struct VehicleParam {

   //VehicleBrand brand ;
    // Car center point is car reference point, i.e., center of rear axle.
    double front_edge_to_center ;
    double back_edge_to_center  ;
    double left_edge_to_center  ;
    double right_edge_to_center ;
    double length ;
    double width ;
    double height ;
    double min_turn_radius;
    double max_acceleration ;
    double max_deceleration ;
    // The following items are used to compute trajectory constraints in planning/control/canbus,
    // vehicle max steer angle
    double max_steer_angle ;
    // vehicle max steer rate; how fast can the steering wheel turn.
    double max_steer_angle_rate ;
    // vehicle min steer rate;
    double min_steer_angle_rate ;
    // ratio between the turn of steering wheel and the turn of wheels
    double steer_ratio ;
    // the distance between the front and back wheels
    double wheel_base ;
    // Tire effective rolling radius (vertical distance between the wheel center and the ground).
    double wheel_rolling_radius ;
    // minimum differentiable vehicle speed, in m/s
    double max_abs_speed_when_stopped ;
  };

  struct SensorHeader {
    // struct publishing time in ms. It is recommended to obtain
    // timestamp_sec from Time::now(), right before calling and publish().
    uint64_t timestamp_ms ;
    // Sequence number for each struct. Each module maintains its own counter for
    // sequence_num, always starting from 1 on boot.
    uint32_t sequence_num ;
    // Lidar Sensor timestamp for millisecond.
    uint64_t lidar_timestamp ;
    // Camera Sensor timestamp for millisecond.
    uint64_t camera_timestamp ;
    // Radar Sensor timestamp for millisecond.
    uint64_t radar_timestamp ;
  };
  struct PbHeader {
    // struct publishing time in ms. It is recommended to obtain
    // timestamp_ms from Time::now(), right before calling and publish().
    uint64_t plan_timestamp = 0 ;
    // peridiction Sensor timestamp for millisecond.
    uint64_t prediction_timestamp = 0 ;
    // Camera Sensor timestamp for millisecond.
    uint64_t camera_timestamp    = 0;
    // chassis Sensor timestamp for millisecond.
    uint64_t chassis_timestamp   = 0;
    // location Sensor timestamp for millisecond.
    uint64_t location_timestamp  = 0;
  };

  struct VehicleConfig
  {
    SensorHeader header;
    VehicleParam vehicle_param;
  };

  struct ADCTrajectoryPoint {
     double x ;  // meters.
     double y ;  // meters.
     double z ;  // height meters.
     double speed ;           // speed,  meters / second
     double acceleration_s ;  // acceleration in s direction
     double curvature ;       // curvature (k = 1/r), unit: (1/meters)
     double curvature_change_rate ; // change of curvature in unit s (dk/ds)
     double relative_time ;         // seconds (relative_time = this_state - timestamp_in_header)
     double theta ;                 // relative to absolute coordinate system
     double accumulated_s ;         // meters, SL，calculated from the first point in this trajectory
     double s ;                     // meters, reference to route SL-coordinate
     double l ;                     //laterl

  };

  struct	ADCPathPoint {
    double x ;  // in meters
    double y ;  // in meters
    double z ;  // in meters
    double curvature ;  // curvature (k = 1/r), unit: (1/meters)
    double heading   ;  // relative to absolute coordinate system
  };

  enum SignalType {
      LEFT_TURN = 1,       //左转灯
      RIGHT_TURN ,         //右转灯
      LOW_BEAM_LIGHT ,     //低光灯
      HIGH_BEAM_LIGHT ,    //远光灯
      FOG_LIGHT ,          //雾灯
      EMERGENCY_LIGHT      //紧急车灯
  };

  struct ADCSignals {

    SignalType signal ;
  };

  struct	EStop {
    // is_estop == true when emergency stop is required
    bool is_estop ;
    string reason ;
  } ;

  struct	PathPoint {
    // coordinates
    double x ; //meter
    double y ; //meter
    double z ; //meter
    double s ; //meter accumulated distance from beginning of the path
    // direction on the x-y plane
    double theta ;
    // curvature on the x-y planning
    double kappa ;
    // derivative of kappa w.r.t s.
    double dkappa ;
    // derivative of derivative of kappa w.r.t s.
    double ddkappa ;
    //lane_id ; // The lane ID where the path point is on
    //derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
    //double x_derivative ;//保留位
    //double y_derivative ;
  };

  struct	Path {
   string name ;
   vector<PathPoint> path_point ;
  };

  struct	TrajectoryPoint {
    PathPoint path_point;  // path point
    double v ;              // in [m/s] linear velocity
    double a ;              // linear acceleration
    double relative_time ;  // relative time from beginning of the trajectory second
  };


  struct VehicleState {
    double x = 0.0;           // m
    double y = 0.0;           // m
    double z = 0.0;           // m
    uint64_t timestamp_ms = 0;
    double roll = 0.0;         // （rad）弧度euler_angles.x
    double pitch = 0.0;        // （rad）弧度euler_angles.y
    double yaw = 0.0;          // （rad）弧度euler_angles.z
    double heading = 0.0;      // （rad）弧度euler_angles.z
    double kappa = 0.0;        //   k = 1/R = w/v
    double linear_velocity = 0.0;      // m / S   chassis.speed_mps 车身坐标系y轴速度
    double angular_velocity = 0.0;     // rad / S^2 车身坐标系绕z轴的的角速度
    double linear_acceleration = 0.0;  // m / S^2   车身坐标系y轴方向的加速度
    GearPosition gear;
    DrivingMode driving_mode ;
    //Pose pose ;
  };

  struct VehiclePositonState {
    PointLLH  point_llh;	      // 全球地理坐标系经纬高
    Point3D   velocity_xyz;          // 东北天方向的速度m/s
    Point3D   acceleration_xyz ;     // 车身坐标系的加速度m/s^2
    Point3D   euler_angles ;         // 符合右手定则角度单位 rad
    Point3D   angular_velocity;      // 车身坐标中的角速度rad/s
  };

  enum TurnSignal {
      TURN_NONE = 0,
      TURN_LEFT ,
      TURN_RIGHT
    };


  struct VehicleSignal {
    TurnSignal turn_signal ;
    // lights enable command
    bool high_beam = false;
    bool low_beam= false;
    bool horn = false;
    bool emergency_light= false;
  };

  enum DeviceAction{
    MOVE_NONE = 0,
    MOVE_UP ,
    MOVE_DOWN
  };
  struct SweeperSignal{
    DeviceAction front_brush = MOVE_NONE;
    double front_brush_speed;
    DeviceAction behind_brush  = MOVE_NONE;
    double behind_brush_speed;
    double fan_Hz;
    double water_rate;
    bool shaker_state = false;
    DeviceAction container_state = MOVE_NONE;

  };
  struct PbTrajectory{

      PbHeader pb_header;
      PointLLH map_orignal_point_llh;	      // 全球地理坐标系经纬高
      VehiclePositonState  vechicle_state;
      bool e_stop = false;        //紧急停车标志
      int32_t trajectory_points_num ; //轨迹点
      vector<TrajectoryPoint> trajectory_points;
      VehicleSignal vehicle_signal;
      SweeperSignal sweeper_signal;
  };

  enum RightOfWayProtectedStatus {
    UNPROTECTED = 0,
    PROTECTED = 1 };

  enum Advice {
          UNKNOWN = 0,
          DISALLOW_ENGAGE ,
          READY_TO_ENGAGE ,
          KEEP_ENGAGED ,
          PREPARE_DISENGAGE
      };

  // This is the engage advice that published by critical runtime modules.
  struct	EngageAdvice {
      Advice advice ;
      string reason ;
  };





  struct TaskStats {
     string name ;
     unsigned long long time_ms ;
  };

  struct LatencyStats {
     unsigned long long total_time_ms ;
     vector<TaskStats> task_stats ;
     unsigned long long init_frame_time_ms ;
  };



  struct SLBoundary {
     double start_s ;
     double end_s   ;
     double start_l ;
     double end_l   ;
  };

  struct	LonCondition {
    double s = 0.0;
    double ds =  0.0;
    double dds = 0.0;
  };

  struct	LatCondition {
    double l = 0.0;
    double dl = 0.0;
    double ddl =  0.0;
  };

  struct	SLPoint {
    double s ;
    double l ;
  };

  struct	FrenetFramePoint {
      double s ;
      double l ;
      double dl ;
      double ddl ;
  };

  struct	SpeedPoint {
    double s ;
    double t ;
    double v ;  // speed (m/s)
    double a ;  // acceleration (m/s^2)
    double da ; // jerk (m/s^3)
  };

  struct ConfigParam {

    double speed_lower_bound;// -0.1 m/s
    double speed_upper_bound;// 11.0 m/s
    double longitudinal_acceleration_lower_bound;//-4.5 m/s^2
    double longitudinal_acceleration_upper_bound;//4.0 m/s^2
    double lateral_acceleration_bound;           //4.0 m/s^2
    double trajectory_time_length;  //8.0 s
    double polynomial_minimal_param;//0.01
    uint32_t num_velocity_sample;//6 //between v + max_dec_a and v + max_acc sample num
    double min_velocity_sample_gap;//1 m/s
    double lattice_epsilon;//10*e-5
    double time_min_density;// 1.0s //st_graph time sample gap
    double default_lon_buffer;//5.0 m //follow or over car safty distance buffer
    uint32_t num_sample_follow_per_timestamp; //3 follow car s orentation sample num
    bool   use_navigation_mode;//false
    double static_obstacle_speed_threshold;//2.0 m/s
    double virtual_stop_wall_height;//2.0 m
    double default_reference_line_width;// 4.0 m
    double trajectory_time_resolution;//0.1s
    double bound_buffer;///0.1 m no using
    double nudge_buffer;///0.3 m no using
    bool   lateral_optimization; //default = false
    double max_s_lateral_optimization;///50.0 m   no using
    double default_delta_s_lateral_optimization;///2.0 m no using
    double lattice_stop_buffer	;//0.02 m for filter
    bool   enable_auto_tuning	;// false
    double decision_horizon	;// 125.0 m
    double trajectory_space_resolution	;// = 1.0 m
    double weight_lon_objective	;// 10.0
    double weight_lon_jerk	;// 1.0
    double weight_lon_collision	;// 5.0
    double weight_centripetal_acceleration ;// 1.5
    double weight_lat_offset	;// 2.0
    double weight_lat_comfort	;// 10.0
    double lat_offset_bound	;// 3.0
    double weight_opposite_side_offset	;// 10.0
    double weight_same_side_offset	;// 1.0
    double longitudinal_jerk_upper_bound	;// 4.0
    double longitudinal_jerk_lower_bound	;//-4.0
    double lateral_jerk_bound	;// 4.0
    double weight_target_speed	;// 1.0
    double weight_dist_travelled	;// 10.0
    double lon_collision_cost_std	;// 0.5
    double lon_collision_yield_buffer	;// 1.0 m
    double lon_collision_overtake_buffer	;// 5.0 m
    double comfort_acceleration_factor	;// 0.5
    double lon_collision_buffer	;// 2.0 m
    double lat_collision_buffer	;// 0.1 m
    double kappa_bound	        ;// 0.2
    bool   enable_trajectory_stitcher	        ;//true
    double replan_lateral_distance_threshold	;// 5.0 m
    double replan_longitudinal_distance_threshold	;// 5.0 m
    bool   enable_lag_prediction	                ;// true
    int32_t  lag_prediction_min_appear_num	        ;// 5
    double lag_prediction_max_disappear_num	;// 3
    double look_forward_time_sec	        ;// 8.0s
    double look_forward_short_distance	;// 90  m
    double look_forward_long_distance	;// 150 m
    double look_backward_distance	        ;// 30m
    bool   enable_change_lane_decider	;// false
    double virtual_stop_wall_length	;// 0.1 m
    bool   align_prediction_time	        ;// false
    double max_collision_distance	        ;// 0.1 m
    bool   ignore_overlapped_obstacle	;// false
    bool   enable_collision_detection	;// false
    int32_t  prediction_message_history_limit	;// 10
    double perception_confidence_threshold	        ;// 0.5
    double lag_prediction_protection_distance	;// 30
    double cost_non_priority_reference_line	;// 0.2
    double planning_upper_speed_limit	        ;// m/s 11.1
    bool   enable_backup_trajectory	        ;// true
    double backup_trajectory_cost	;// 1000.0
    bool   use_planning_fallback	;// true
    bool   planning_test_mode	;// false
    bool   estimate_current_vehicle_state	        ;// true
    int32_t  planning_loop_rate	;// 10
    bool   enable_record_debug	;// true
    bool   publish_estop	        ;// false
    double trajectory_time_high_density_period	;// 1.0s
    bool	enable_stitch_last_trajectory	;// true
    bool	enable_map_reference_unify	;// true
    double	max_stop_distance_obstacle	;// 10.0 m
    double	min_stop_distance_obstacle	;// 6.0 m
    double	st_max_t	;// 8.0 S
    double	st_max_s	;// 100.0 m
    double	max_stop_speed	;// 0.2 m/s
    double	default_cruise_speed	;// 9.9 m/s
    bool        use_multi_thread_to_add_obstacles;//false
    double	destination_check_distance	 ;// 5.0
    int32_t	destination_obstacle_id	;// 0
    bool	reckless_change_lane	;/// false no using
    double	change_lane_success_freeze_time	;/// 3.0 no using
    double	change_lane_fail_freeze_time	;/// 3.0 no using
    double	segment_length	;// 25.0 m
    double	interval_length	;// 5.0 m
    double	discretion_length	;// 0.25 m

    double      clean_max_dis   ;//= 16.0 m;
    double      clean_max_width ;//= 2.6 m;
    double      clean_min_width ;//= -2.0 m;
    double      clean_max_evaluate_width ;// 4.0;
    double      clean_ignore_dis ;// 2.5;
    double      clean_effect_dis ;// 12;

    double      ultra_front_safe_dis ;// 2.0;
    double      laterl_front_safe_dis  ;// 0.5;
    double      laterl_back_safe_dis ;// 0.5;
    double      ultra_back_safe_dis  ;// 0.4;
    double      weight_clean_garbage ;// 30

    int         planning_make_prediction_max_points_num;//80
    double      gps_status_max_still_time;//8.0 S
    double      ultrasonic_find_danger_min_still_time;//0.3 S

    bool enable_ploygon_checking;
    bool enable_data_record;
    bool enable_data_playback;
    double slide_trans_dis;
    int prediction_point_num;
    double sample_width;
    double max_angle_velocity;//deg/sec
    double max_beyond_side_buffer;
    double max_slide_gap;
    double cost_non_slide_reference_line;
    double default_slide_center_to_right_dis;
    double obstacle_filter_buffer;
    double ultra_stop_wait_time;
    double decider_sample_dis;
    double combine_stop_result_dis;
    double valid_stop_decider_dis;
    double valid_passing_obs_dis;
    double per_error_dis;



  };


  enum Scenario {
      SCENARIO_UNKNOWN = 0,
      CRUISE_UNKNOWN ,
      CRUISE_URBAN ,
      CRUISE_HIGHWAY ,
      JUNCTION_UNKNOWN ,
      JUNCTION_TRAFFIC_LIGHT ,
      JUNCTION_STOP_SIGN
    };


  struct SpeedControl {
    string name ;
    Polygon polygon ;
    double speed_limit;
  };

  struct SpeedControls {
      vector<SpeedControl> speed_control;
  };

  enum GarbageType{
    UNKNOWN_WASTE = 0,
    LEAF,
    PEEL,
    WASTE_PAPER,
    WASTE_BOTTLE,
    PLASTIC_BAG
  };

  struct Garbage{
    int     id;
    Point3D position;
    vector<Point3D> polygons;
    double  confidence;
    double  density;
    double  length;
    double  width;
    double  yaw;
    double  tracking_time;
    GarbageType garbage_type;
  };

  struct CleanTarget{
    uint64_t timestamp;//ms
    vector<Garbage> Garbages;
  };


  struct Ultrasonic{

    int id;
    bool valid = false;
    double distance; //0.3~2.6 m
  };

  struct UltrasonicSense{

    uint64_t time_stamp;//ms
    vector<Ultrasonic>  front;
    vector<Ultrasonic>  back;
    vector<Ultrasonic>  left;
    vector<Ultrasonic>  right;

  };

  enum CleanMode{
    DRIVE_MODE = 0,
    BORDER_MODE ,
    SHOW_MODE
  };
  enum RouteType{
    ROUTE_UNKNOWN = 0,
    SINGLE_ROUTE,
    SINGLE_SLIDE,
    DOUBLE_ROUTE,
    DOUBLE_SINGLE_SLIDE
  };
  struct SoftCommand{
    CleanMode clean_mode = DRIVE_MODE;//default border clean
    bool auto_parking = false;//default disable auto parking

  };

//}//end namespace planning



/*****************************************************************************************/
/********** perception *******************************************************************/
namespace perception {

  enum ObstacleType{
        UNKNOWN = 0,
        UNKNOWN_MOVABLE ,
        UNKNOWN_UNMOVABLE,
        PEDESTRIAN , // Pedestrian, usually determined by moving behaviour.
        BICYCLE ,  // bike, motor bike
        VEHICLE   // Passenger car or truck.

  };
  struct Point {
    double x ;
    double y ;
    double z ;
  };

  struct Velocity{

    double x ; //m/s
    double y ; //m/s
    double z ; //m/s
  };

  enum ConfidenceType {
      CONFIDENCE_UNKNOWN = 0,
      CONFIDENCE_CNN = 1,
      CONFIDENCE_RADAR = 2,
  };

  struct PerceptionObstacle {
    int32_t     id ;         // obstacle ID.
    Point     position ;   // obstacle position in the world coordinate system.
    double    theta ;      // heading in the world coordinate system.
    Velocity  velocity ;   // obstacle velocity.

    // Size of obstacle bounding box.
    double length ;  // obstacle length.
    double width ;   // obstacle width.
    double height ;  // obstacle height.
    int32_t polygon_num ;//多边形边数
    vector<Point> polygon_point ;   // obstacle corner points.
    double  tracking_time ;         // duration of an obstacle since detection in s.
    ObstacleType type ;             // obstacle type
    double confidence = 1.0;        // [default = 1.0];
  };

  enum Color {
      Unknown = 0,
      RED = 1,
      YELLOW = 2,
      GREEN = 3,
      BLACK = 4
  };
  struct TrafficLightBox {
       int32_t x ;// 1;
       int32_t y ;// 2;
       int32_t width ;// 3;
       int32_t height ;// 4;
       Color color ;// 5;
       bool selected ;// 6;
  };

  struct TrafficLightDebug {
       TrafficLightBox cropbox ;// 1;
       vector<TrafficLightBox> box ;// 2;
       int32_t  signal_num ;// 3;
       int32_t  valid_pos ;// 4;
       double ts_diff_pos ;// 5;
       double ts_diff_sys ;// 6;
       int32_t  project_error ;// 7;
       double distance_to_stop_line ;// 8;
       int32_t  camera_id ;// 9;
  };

  struct TrafficLight {

       Color color ; //1
       int32_t id ;    //2 Traffic light string-ID in the map data.
       //3 How confidence about the detected resultsbetween 0 and 1.
       double confidence = 1.0 ;
       //4 Duration of the traffic light since detected.
       double tracking_time ;
  };

  struct TrafficLightDetection {
       uint64_t camera_timestamp ;              // 1. ms;
       int32_t  frame_num = 0;                     // 2. 侦测到的数据帧数
       vector<TrafficLight> traffic_light ;   // 3.
      // bool contain_lights ;
  };

} //namespace perception

/*******************************************************************************************/
/************************prediction*********************************************************/
namespace prediction {

  struct Trajectory {
    double probability ;  // probability of this trajectory
    int32_t  trajectory_point_num;//轨迹点数量
    vector<TrajectoryPoint> trajectory_point ;
  };


  enum ObstacleIntent {
      UNKNOWN = 0,
      STOP = 1,
      STATIONARY = 2,
      MOVING = 3,
      CHANGE_LANE = 4,
      LOW_ACCELERATION = 5,
      HIGH_ACCELERATION = 6,
      LOW_DECELERATION = 7,
      HIGH_DECELERATION = 8,
    };

  enum Intent {
        INTENT_UNKNOWN = 0,
        STOPCAR ,
        CRUISE ,
        INTENT_CHANGE_LANE,
      };

 struct PredictionObstacle {
    perception::PerceptionObstacle perception_obstacle ;
    uint64_t timestamp ;  // GPS time in seconds
    // the length of the time for this prediction (e.g. 10s)
    double predicted_period ;
    // can have multiple trajectories per obstacle
    int32_t trajectory_num;//轨迹条数
    vector<Trajectory> trajectory ;//repeated
  };


 struct PredictionObstacles {

    // make prediction for multiple obstacles
    int32_t prediction_obstacle_num;//感知到的障碍物数量
    vector<PredictionObstacle> prediction_obstacle ;
    //gps week num
    //uint32_t gps_week;
    //gps week round num ms
    //uint64_t gps_week_ms;
    // start timestamp
    uint64_t start_timestamp ;
    // end timestamp
    uint64_t end_timestamp ;
  };



} //namespace prediction

/************************************************************************************************/
/************************************************************************************************/


namespace canbus {

  enum ErrorCode {
    NO_ERROR = 0,
    CMD_NOT_IN_PERIOD = 1,  // control cmd not in period
    // car chassis report error, like steer, brake, throttle, gear fault
    CHASSIS_ERROR = 2,
    MANUAL_INTERVENTION = 3,  // human manual intervention
    // receive car chassis can frame not in period
    CHASSIS_CAN_NOT_IN_PERIOD = 4,
    UNKNOWN_ERROR = 5
  };


  struct License {
    string vin ;
  };

  enum GpsQuality {
    FIX_NO = 0,
    FIX_2D = 1,
    FIX_3D = 2,
    FIX_INVALID = 3,
  };

  struct ChassisGPS {
    double latitude ;
    double longitude;
    bool gps_valid  ;

    int32_t year ;
    int32_t month ;
    int32_t day ;
    int32_t hours ;
    int32_t minutes ;
    int32_t seconds ;
    double compass_direction ;
    double pdop ;
    bool is_gps_fault ;
    bool is_inferred ;

    double altitude ;
    double heading ;
    double hdop ;
    double vdop ;
    GpsQuality quality ;
    int32_t num_satellites ;
    double gps_speed ;
  };
  enum WheelSpeedType {
    FORWARD = 0,
    BACKWARD = 1,
    STANDSTILL = 2,
    INVALID = 3
  };

  struct WheelSpeed {

    bool is_wheel_spd_rr_valid ;       // = 1 [default = false];
    WheelSpeedType wheel_direction_rr ;//= 2 [default = INVALID];
    double wheel_spd_rr ;              //= 3 [default = 0.0];
    bool is_wheel_spd_rl_valid ;       //= 4 [default = false];
    WheelSpeedType wheel_direction_rl ;//= 5 [default = INVALID];
    double wheel_spd_rl ;              //= 6 [default = 0.0];
    bool is_wheel_spd_fr_valid ;       //= 7 [default = false];
    WheelSpeedType wheel_direction_fr ;//= 8 [default = INVALID];
    double wheel_spd_fr ;              //= 9 [default = 0.0];
    bool is_wheel_spd_fl_valid ;       //= 10 [default = false];
    WheelSpeedType wheel_direction_fl ;//= 11 [default = INVALID];
    double wheel_spd_fl ;              //= 12 [default = 0.0];
  };

  struct Sonar {
    double range ;  // Meter
    Point3D translation ;  // Meter
    Quaternion rotation ;
  };

  struct Surround {
    bool cross_traffic_alert_left ;//= 1;
    bool cross_traffic_alert_left_enabled ;//= 2;
    bool blind_spot_left_alert ;//= 3;
    bool blind_spot_left_alert_enabled ;//= 4;
    bool cross_traffic_alert_right ;//= 5;
    bool cross_traffic_alert_right_enabled ;//= 6;
    bool blind_spot_right_alert ;//= 7;
    bool blind_spot_right_alert_enabled ;//= 8;
    double sonar00 ;//= 9;
    double sonar01 ;//= 10;
    double sonar02 ;//= 11;
    double sonar03 ;//= 12;
    double sonar04 ;//= 13;
    double sonar05 ;//= 14;
    double sonar06 ;//= 15;
    double sonar07 ;//= 16;
    double sonar08 ;//= 17;
    double sonar09 ;//= 18;
    double sonar10 ;//= 19;
    double sonar11 ;//= 20;
    bool sonar_enabled ;//= 21;
    bool sonar_fault ;//= 22;
    double sonar_range ;//= 23;
    Sonar sonar ;//= 24;
  };

 struct Chassis { 
    // 1.chassis also needs it own sending timestamp
     uint64_t timestamp_ms;
    // 2.Vehicle Speed in meters per second.
      float speed_mps ;//necesarry
    // 3.engine work status
     bool engine_started = false;
    // 4.Engine speed in RPM.
     float engine_rpm ;
    // 5.Vehicle odometer in meters.
     float odometer_m ;
    // 6.battery range in kilometers.
     int32_t battery_range_km ;
    // 7.Real throttle location in [%], ranging from 0 to 100.
     float throttle_percentage;
    // 8.Real brake location in [%], ranging from 0 to 100.
     float brake_percentage ;
    // 9.Real steering location in [%], ranging from 0 to 100.
     float steering_percentage ;
    // 10.Applied steering torque in [Nm].
     float steering_torque_nm ;
    // 11.Parking brake status.
     bool parking_brake = false;
    // 12.wiper status
     bool wiper = false;
    // 13.DrivingMode [default = COMPLETE_MANUAL];
     DrivingMode driving_mode ;
    // 14.gear_location
     GearPosition gear_location ;
    // 15.int32_t chassis_error_mask ;
     VehicleSignal signal ;
    // 16.data refresh or not ?
     bool has_data = false;
    //17.splice angle
     double splice_angle;

  };

} //namespace canbus
/***********************************************************************/
/***********************************************************************/


namespace TrafficRuleStruct {

  struct BacksideVehicleConfig {
    // a vehicle is considered within current lane if it is behind the ADC
    // and its lateral difference is less than this number.
    double backside_lane_width ;//= 1 [ default=4.0 ];
  };

  struct ChangeLaneConfig {
    double min_overtake_distance ;//= 1 [ default=10.0 ];
    double min_overtake_time ;//= 2 [ default=2.0 ];
    bool  enable_guard_obstacle ;//= 3 [ default=false ];
    double guard_distance ;//= 4 [ default=100.0 ];
    double min_guard_speed ;//= 5 [ default=1.0 ];
  };

  struct CreepConfig {
    bool enabled ;//= 1;
    // stop distance(m) to the stop line of next lane overlap while creeping
    double creep_distance_to_stop_line ;//= 2 [ default=1.0 ]; // meter
    // stop distance(m) to the stop line of next lane overlap while creeping
    double stop_distance ;//= 3 [ default=0.5 ]; // meter
    double speed_limit ;//= 4 [ default=1.0 ]; // m/s
    // max distance(m) to the stop line to be considered as a valid stop for creap
    double max_valid_stop_distance ;//= 5 [ default=0.3 ]; // meter
    // min boundary t to ignore obstacles while creeping
    double min_boundary_t ;//= 6 [ default=6.0 ]; // second
    // min boundary s to ignore obstacles while creeping
    double min_boundary_s ;//= 7 [ default=3.0 ]; // meter
  };

  struct CrosswalkConfig {
    // stop distance from stop line of crosswalk
    double stop_distance ;//= 1 [ default=1.0 ]; // meter
    // max deceleration
    double max_stop_deceleration ;//= 2 [ default=4.0 ];
    // min s_distance for adc to be considered have passed crosswalk (stop_line_end_s)
    double min_pass_s_distance ;//= 3 [ default=1.0 ]; // meter
    // max speed(m/s) to be considered as a valid stop
    double max_stop_speed ;//= 4 [ default=0.3 ]; // m/s
    // max distance(m) to the stop line to be considered as a valid stop
    double max_valid_stop_distance ;//= 5 [ default=3.0 ]; // meter
    // expand s_distance for pedestrian/bicycle detection
    double expand_s_distance ;//= 6 [ default=2.0 ]; // meter
    // strick stop rule within this l_distance
    double stop_strick_l_distance ;//= 7 [ default=4.0 ]; // meter
    // loose stop rule beyond this l_distance
    double stop_loose_l_distance;// = 8 [ default=5.0 ]; // meter
    // stop timeout for bicycles/pedestrians which are not moving
    double stop_timeout ;//= 9 [ default=10.0 ]; // second
  };

  struct DestinationConfig {
    // flag to enable pullover upon arriving destination
    bool enable_pull_over ;//= 1 [ default=false ];
    // stop distance from destination line
    double stop_distance ;//= 2 [ default=0.5 ]; // meter
    // distance to stop point to start planning pull over
    // TODO(all): must be in sync with the same config in PULL_OVER will remove
    double pull_over_plan_distance ;//= 3 [default=35.0];
  };

  struct FrontVehicleConfig {
    // Waiting time(sec) before starting sidepass vehicle
    bool enable_side_pass ;//= 1 [ default=true ];
    // within this s_distance to ADC to compute SIDEPASS
    double side_pass_s_threshold ;//= 2 [ default=15.0 ]; // meter
    // within this l_distance(+/-) to ADC to considere SIDEPASS
    double side_pass_l_threshold ;//= 3 [ default=1.0 ];  // meter
    double side_pass_wait_time ;//= 4 [ default=30.0]; // second
    double nudge_l_buffer ;//= 5 [ default=0.5]; // meter
  };

  struct KeepClearConfig {
    // min s_distance to be considered have passed keep_clear (stop_line_end_s)
    bool enable_keep_clear_zone ;//= 1 [ default=true ];
    bool enable_junction ;//= 2 [ default=true ];
    double min_pass_s_distance;// = 3 [ default=2.0 ]; // meter
  };

  struct PullOverConfig {
    // stop distance from stop line
    double stop_distance ;//= 1 [default=0.5];
    // max speed(m/s) to be considered as a valid stop
    double max_stop_speed ;//= 2 [ default=0.3 ]; // m/s
    // max distance(m) to the stop line to be considered as a valid stop
    double max_valid_stop_distance ;//= 3 [ default=3.0 ]; // meter
    // max deceleration
    double max_stop_deceleration;// = 4 [ default=2.5 ];
    // min s_distance for adc to be considered have passed crosswalk (stop_line_end_s)
    double min_pass_s_distance ;//= 5 [ default=1.0 ]; // meter
    // l-distance buffer to road/lane boundary
    double buffer_to_boundary ;//= 6 [default=0.5];
    // distance to stop point to start planning pull over
    double plan_distance ;//= 7 [default=35.0];
    // s_distance to beteeen start and stop points
    double operation_length ;//= 8 [ default=30.0 ]; // meter
    // max s_distance to check ahead while planning pull over
    double max_check_distance ;//= 9 [ default=60.0]; // meter
    // max failure count before changing to stop in-lane
    uint32_t max_failure_count ;//= 10 [ default=10];
  };

  struct ReferenceLineEndConfig {
    // stop distance from refrence line end
    double stop_distance ;//= 1 [ default=0.5 ]; // meter
    double min_reference_line_remain_length ;//= 2 [ default=50.0 ];
  };

  struct ReroutingConfig {
    // should not rerouting more frequent than this number
    double cooldown_time ;//= 1 [ default=3.0 ];  // seconds
    double prepare_rerouting_time ;//= 2 [ default=2.0 ]; // seconds
  };

  struct SignalLightConfig {
    // 1.stop distance from stop line
    double stop_distance ;// [ default=1.0 ]; // meter
    // 2.max deceleration
    double max_stop_deceleration ;// [ default=4.0 ];
    // 3.min s_distance for adc to be considered have passed signal_light (stop_line_end_s)
    double min_pass_s_distance ;//[ default=4.0 ] meter
    // 4.treat yellow light as red when deceleration (abstract value in m/s^2)
    // is less than this threshold; otherwise treated as green light
    double max_stop_deacceleration_yellow_light ;// [ default=3.0 ];
    // 5.
    double signal_expire_time_sec ;// [ default=5.0 ] second
    // 6.
    CreepConfig righ_turn_creep ;//;
    // consider the signal msg is expired if its timestamp over this threshold(s)

  };

  struct StopSignConfig {
    // stop distance from stop line of stop sign
    double stop_distance ;//= 1 [ default=1.0 ]; // meter
    // min s_distance for adc to be considered have passed stop_sign (stop_line_end_s)
    double min_pass_s_distance ;//= 2 [ default=1.0 ]; // meter
    // max speed(m/s) to be considered as a valid stop
    double max_stop_speed ;//= 3 [ default=0.3 ]; // m/s
    // max distance(m) to the stop line to be considered as a valid stop
    double max_valid_stop_distance ;//= 4 [ default=3.0 ]; // meter
    // min time(seconds) to be considered as a full stop at stop sign
    double stop_duration;// = 5 [ default=1.0 ]; // seconds
    // max speed(m/s) for watch vehicles to be considered as
    // a valid stop.(this check is looser than adc)
    double watch_vehicle_max_valid_stop_speed ;//= 6 [ default=0.5 ]; // m/s
    // max stop distance for watch vehicles to be considered as
    // a valid stop.(this check is looser than adc)
    double watch_vehicle_max_valid_stop_distance ;//= 7 [ default=5.0 ]; // meter
    // timeout threshold while monitoring/waiting for other obstacles
    double wait_timeout ;//= 8 [ default=8.0 ]; // sec
    CreepConfig creep ;//= 9;
  };
  enum RuleId {
    BACKSIDE_VEHICLE = 1,
    CHANGE_LANE = 2,
    CROSSWALK = 3,
    DESTINATION = 4,
    FRONT_VEHICLE = 5,
    KEEP_CLEAR = 6,
    PULL_OVER = 7,
    REFERENCE_LINE_END = 8,
    REROUTING = 9,
    SIGNAL_LIGHT = 10,
    STOP_SIGN = 11
  };

  struct OneOfConfig {
        BacksideVehicleConfig backside_vehicle    ;// 01
        ChangeLaneConfig change_lane              ;// 02
        CrosswalkConfig crosswalk                 ;// 03
        DestinationConfig destination             ;// 04
        FrontVehicleConfig front_vehicle          ;// 05
        KeepClearConfig keep_clear                ;// 06
        PullOverConfig pull_over                  ;// 07
        ReferenceLineEndConfig reference_line_end ;// 08
        ReroutingConfig rerouting                 ;// 09
        SignalLightConfig signal_light            ;// 10
        StopSignConfig stop_sign                  ;// 11
      };
  // TODO(all) migrate all other traffic rules to pb config
  struct TrafficRuleConfig {

    RuleId rule_id             ;
    bool enabled               ;
    //int32_t config_subsquence    ;
    OneOfConfig one_of_config ;
  };

  struct TrafficRuleConfigs {
    vector<TrafficRuleConfig> config;
  };



} //namespace TrafficeRule

namespace common {
  enum Status {


    // No error, reutrns on success.
      OK = 100,
      // Control module error codes start from here.
      CONTROL_ERROR = 1000,
      CONTROL_INIT_ERROR = 1001,
      CONTROL_COMPUTE_ERROR = 1002,
      // Canbus module error codes start from here.
      CANBUS_ERROR = 2000,
      CAN_CLIENT_ERROR_BASE = 2100,
      CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101,
      CAN_CLIENT_ERROR_FRAME_NUM = 2102,
      CAN_CLIENT_ERROR_SEND_FAILED = 2103,
      CAN_CLIENT_ERROR_RECV_FAILED = 2104,
      // Localization module error codes start from here.
      LOCALIZATION_ERROR = 3000,
      LOCALIZATION_ERROR_MSG = 3100,
      LOCALIZATION_ERROR_LIDAR = 3200,
      LOCALIZATION_ERROR_INTEG = 3300,
      LOCALIZATION_ERROR_GNSS = 3400,
      // Perception module error codes start from here.
      PERCEPTION_ERROR = 4000,
      PERCEPTION_ERROR_TF = 4001,
      PERCEPTION_ERROR_PROCESS = 4002,
      PERCEPTION_FATAL = 4003,
      // Prediction module error codes start from here.
      PREDICTION_ERROR = 5000,
      // Planning module error codes start from here
      PLANNING_ERROR = 6000,
      // HDMap module error codes start from here
      HDMAP_DATA_ERROR = 7000,
      // Routing module error codes
      ROUTING_ERROR = 8000,
      ROUTING_ERROR_REQUEST = 8001,
      ROUTING_ERROR_RESPONSE = 8002,
      ROUTING_ERROR_NOT_READY = 8003,
      // Indicates an input has been exhausted.
      END_OF_INPUT = 9000,
      // HTTP request error codes.
      HTTP_LOGIC_ERROR = 10000,
      HTTP_RUNTIME_ERROR = 10001,
      // Relative Map error codes.
      RELATIVE_MAP_ERROR = 11000, // general relative map error code
      RELATIVE_MAP_NOT_READY = 11001,
      // Driver error codes.
      DRIVER_ERROR_GNSS = 12000,
      DRIVER_ERROR_VELODYNE = 13000,

  };
}// namespace common



struct TargetLane {
  // lane id
  int32_t  id ;//1;
  double start_s ;//2;  // in meters
  double end_s ;//3;    // in meters
  double speed_limit ;//4;  // in m/s
};

struct ObjectIgnore {

};

enum StopReasonCode {
  STOP_REASON_HEAD_VEHICLE = 1,
  STOP_REASON_DESTINATION = 2,
  STOP_REASON_PEDESTRIAN = 3,
  STOP_REASON_OBSTACLE = 4,
  STOP_REASON_PREPARKING = 5,
  STOP_REASON_SIGNAL = 100, // only for red signal
  STOP_REASON_STOP_SIGN = 101,
  STOP_REASON_YIELD_SIGN = 102,
  STOP_REASON_CLEAR_ZONE = 103,
  STOP_REASON_CROSSWALK = 104,
  STOP_REASON_CREEPER = 105,
  STOP_REASON_REFERENCE_END = 106, // end of the reference_line
  STOP_REASON_YELLOW_SIGNAL = 107, // yellow signal
  STOP_REASON_PULL_OVER = 108 // pull over

};

struct ObjectStop {
  StopReasonCode reason_code ;//1;
  double distance_s ;         //2;  // in meters
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point ;//=3;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading ;            //4;
  vector<int32_t> wait_for_obstacle ;//5;

};

enum NudgeType {
  LEFT_NUDGE = 1,   // drive from the left side of the obstacle
  RIGHT_NUDGE = 2,  // drive from the right side of the obstacle
  NO_NUDGE = 3      // No nudge is set.
};

// dodge the obstacle in lateral direction when driving
struct ObjectNudge {

  NudgeType nudge_type ;//= 1;
  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
  double distance_l ;//= 2;

};

struct ObjectYield {
  double distance_s ;//1;  // minimum longitudinal distance in meters
  PointENU fence_point ;//2;
  double fence_heading ;//3;
  double time_buffer ;//4;  // minimum time buffer required after the obstacle reaches the intersect point.


};

struct ObjectFollow {
  double distance_s ;//1;  // minimum longitudinal distance in meters
  PointENU fence_point ;//2;
  double fence_heading ;//3;

};

struct ObjectOvertake {
  double distance_s = 1;  // minimum longitudinal distance in meters
  PointENU fence_point ;//2;
  double fence_heading ;//3;
  double time_buffer ;//4;  // minimum time buffer required before the obstacle reaches the intersect point.

};
enum PassType {
  LEFT = 1,
  RIGHT = 2
};
struct ObjectSidePass {

  PassType pass_type ;

};

// unified object decision while estop
struct ObjectAvoid {

};

struct ObjectDecisionType {


    ObjectStop  stop ;           //7;
    ObjectYield yield ;          //6;
    ObjectFollow follow ;        //5;
    ObjectOvertake overtake ;    //4;
    ObjectNudge nudge ;          //3;
    ObjectSidePass sidepass ;    //2;
    ObjectIgnore ignore ;        //1;

    //ObjectSidePass sidepass ;//7;
    //ObjectAvoid avoid ;//8;

    bool has_ignore = false;
    bool has_stop   = false;
    bool has_nudge  = false;
    bool has_yield  = false;
    bool has_follow = false;
    bool has_overtake = false;
    bool has_sidepass = false;
    //bool has_avoid = false;
    int object_tag_case = 0;

};


struct ObjectDecision {
  int32_t id ;//1;
  int32_t perception_id ;//2;
  vector<ObjectDecisionType> object_decision ;//3;
};

struct ObjectDecisions {
  vector<ObjectDecision> decision ;//1;
};

struct MainStop {
  StopReasonCode reason_code ;//1;
  string reason ;//2;
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point ;//3;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading ;//4;
  //apollo.routing.ChangeLaneType change_lane_type = 5;
};


struct EmergencyStopHardBrake {
};

struct EmergencyStopCruiseToStop {
};

enum ReasonCode {
  ESTOP_REASON_INTERNAL_ERR = 1,
  ESTOP_REASON_COLLISION = 2,
  ESTOP_REASON_ST_FIND_PATH = 3,
  ESTOP_REASON_ST_MAKE_DECISION = 4,
  ESTOP_REASON_SENSOR_ERROR = 5
};

 struct OneOfEmergencyStopType {
  EmergencyStopHardBrake hard_brake ;  // hard brake
  EmergencyStopCruiseToStop cruise_to_stop ;  // cruise to stop
};

struct MainEmergencyStop {
  // Unexpected event happened, human driver is required to take over

  ReasonCode reason_code ;
  string reason ;
  //OneOfEmergencyStopType emergency_stop_type;

};

struct MainCruise {
  // cruise current lane
  //apollo.routing.ChangeLaneType change_lane_type = 1;
};

// This struct is deprecated
struct MainChangeLane {

  PassType pass_type ;
  vector<TargetLane> default_lane;
  MainStop default_lane_stop ;
  MainStop target_lane_stop ;
};

struct MainMissionComplete {
  // arrived at routing destination
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point ;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading ;
};

struct MainNotReady {
  // decision system is not ready.
  // e.g. wait for routing data.
  bool has_reason = false;
  string reason ;
};

struct MainParking {
};

struct OneOfDecisionTask {
  //MainCruise cruise ;
  MainStop stop ;
  MainEmergencyStop estop ;
  MainChangeLane change_lane ;// [deprecated=true];
  MainMissionComplete mission_complete ;
  MainNotReady not_ready ;//;= 7;
  //MainParking parking ;//= 8;
};

struct MainDecision {
  OneOfDecisionTask task;
  vector<TargetLane> target_lane ;//= 5 [deprecated = true];
};

struct DecisionResult {
  MainDecision main_decision ;
  ObjectDecisions object_decision ;
  VehicleSignal vehicle_signal ;
};

namespace ADC {

  enum TrajectoryType {
      UNKNOWN = 0,
      NORMAL = 1,
      PATH_FALLBACK = 2,
      SPEED_FALLBACK = 3
    };

}

namespace common {
  struct  Id {
    int32_t id ;
  };

}//namespace ommon


struct	ADCTrajectory {
  SensorHeader header ;                //存放各种时间戳
  double total_path_length ;     // in meters
  double total_path_time ;       // in seconds
  bool   is_replan =  false ;    // is_replan == true mean replan triggered
  EStop  estop ;                 //急停标识
  SensorHeader routing_header ;        // the routing used for current planning result
  GearPosition              gear ;                 // Specify trajectory gear
  DecisionResult            decision ;
  LatencyStats              latency_stats ;
  EngageAdvice              engage_advice ;         //离合建议
  VehicleSignal             signal ;
  vector<Point3D>           point;                   //关键点
  vector<common::Id>        lane_id ;                //lane id along reference line
  vector<PathPoint>         path_point ;             //reference line path data
  ADC::TrajectoryType       trajectory_type = ADC::UNKNOWN ;//]21 [ default = UNKNOWN];
  vector<TrajectoryPoint>   trajectory_point ;     //路径点集合 path + speed
  RightOfWayProtectedStatus right_of_way_status ;
  // set the engage advice for based on current planning result.
  // the region where planning cares most
  // Debug debug ;

};



namespace ScenarioConfig{

  enum ScenarioType {
      LANE_FOLLOW = 0,  // default scenario
      CHANGE_LANE = 1,
      SIDE_PASS = 2,  // go around an object when it blocks the road
      APPROACH = 3,  // approach to an intersection
      INTERSECTION_STOP_SIGN_FOUR_WAY = 4,
      INTERSECTION_STOP_SIGN_ONE_OR_TWO_WAY = 5,
      INTERSECTION_TRAFFIC_LIGHT_LEFT_TURN = 6,
      INTERSECTION_TRAFFIC_LIGHT_RIGHT_TURN = 7,
      INTERSECTION_TRAFFIC_LIGHT_GO_THROUGH = 8,
    };


}
namespace localization {

  struct LocalizationEstimate {

    int32_t  data_size; //vector数据实际存储的大小
    vector<Pose> pose_30 ;    //= 2;

  };

  struct Uncertainty {
    // Standard deviation of position, east/north/up in meters.
    Point3D position_std_dev ;//= 1;

    // Standard deviation of quaternion qx/qy/qz, unitless.
    Point3D orientation_std_dev ;//= 2;

    // Standard deviation of linear velocity, east/north/up in meters per second.
    Point3D linear_velocity_std_dev ;//= 3;

    // Standard deviation of linear acceleration, right/forward/up in meters per
    // square second.
    Point3D linear_acceleration_std_dev ;//= 4;

    // Standard deviation of angular velocity, right/forward/up in radians per
    // second.
    Point3D angular_velocity_std_dev ;//= 5;

    // TODO: Define covariance items when needed.
  };




  enum MeasureState {
    NOT_VALID = 0,
    NOT_STABLE = 1,
    OK = 2,
    VALID = 3
  };

  struct LocalizationStatus {
    uint64_t timestamp_ms ;// 1
    MeasureState fusion_status ;//= 2;
    MeasureState gnss_status ;//= 3;
    MeasureState lidar_status ;//= 4;
    // The time of pose measurement, seconds since the GPS epoch (Jan 6, 1980).
    double measurement_time ;//= 5;  // In seconds.
  };

}// namespace localization



namespace PlanningStatusStruct {

  /*
    This file defines the data types that represents the internal state of the planning module.
    It will not be refreshed in each planning cycle.
  */
  enum RoadStatus {
    UNKNOWN = 1,
    IN_OPERATION = 2,
    DONE = 3,
    DISABLED = 4,
    DRIVE ,
    STOP ,
    WAIT ,
    SIDEPASS ,
    CREEP ,
    STOP_DONE ,
    IN_CHANGE_LANE , // during change lane state
    CHANGE_LANE_FAILED , // change lane failed
    CHANGE_LANE_SUCCESS  // change lane failed
  };

  struct ChangeLaneStatus {

    RoadStatus status;
    // the id of the route segment that the vehicle is driving on
    int32_t path_id;
    // the time stamp when the state started.
    uint64_t timestamp;
  };

  struct StopTimer {
    int32_t obstacle_id;
    // the timestamp when start stopping for the crosswalk
    double stop_time ;
  };

  struct CrosswalkStatus {
    int32_t crosswalk_id ;
    // the timestamp when start stopping for the crosswalk
    vector<StopTimer> stop_timers ;
  };

  enum Reason {
    ONDESTINATION // upon arriving destination
  };

  struct  PullOverStatus {

    bool in_pull_over = false;
    RoadStatus status;
    PointENU inlane_dest_point ;
    PointENU start_point ;
    PointENU stop_point  ;
    double stop_point_heading ;
    Reason reason ;
    double status_set_time ;
  };

  struct  ReroutingStatus {
    double last_rerouting_time ;
  };

  struct RightOfWayStatus {
    // whether the vehicle has right of way in junction
    std::map<int, bool> junction ;
  };

  struct  SidePassStatus {
    // a sidepass sequence includes:
    // driving -> wait -> sidepass -> driving
    RoadStatus status ;
    double wait_start_time ;
    string pass_obstacle_id ;
    PassType pass_side ;
  };

  struct LaneWatchVehicles {
    int32_t lane_id ;
    vector<int32_t> watch_vehicles ;
  };

  struct StopSignStatus {
    int32_t stop_sign_id ;
    RoadStatus status ;
    double stop_start_time ;
    vector<LaneWatchVehicles> lane_watch_vehicles;
  };

  struct DestinationStatus {
      bool has_passed_destination = false;
  };

  struct PlanningStatus {
    ChangeLaneStatus  change_lane ;
    CrosswalkStatus   crosswalk ;
    EngageAdvice      engage_advice ;
    ReroutingStatus   rerouting ;
    RightOfWayStatus  right_of_way ;
    SidePassStatus    side_pass ;
    StopSignStatus    stop_sign ;
    DestinationStatus destination  ;
    PullOverStatus    pull_over ;
  };

} //namespace PlanningStatus

struct SampleRecommendation
{
    enum DriveBias{
        NONE = 0,
        RIGHT_PASS ,
        RIGHT_CENTER_PASS,
        LEFT_PASS,
        LEFT_CENTER_PASS,
        LEFT_RIGHT,
        STOP_DRIVR
    };
   DriveBias pass_type = NONE;
   SLPoint sl_point;
};


}//end namespace planning
#if 1
struct MSFLoutput{
     char    head;  // 帧头
     unsigned long long timestamp; //uinx timestamp unit ms
     int    gpsWeek;
     double gpsSec;

     double  lat;  // degree
     double  lon;  // degree
     double  height;    // m
     double  velEast;   // m/s
     double  velNorth;  // m/s
     double  velUp;     // m/s
     double  pitch; // degree, 向上为正,   [-90,  90]
     double  roll;  // degree, 右倾为正,    [-180, 180
     double  yaw;   // degree, 北偏东为正,   [0,  360]

     double  sigmaLat;   // unit:m
     double  sigmaLon;   // unit:m
     double  sigmaHeight;// unit:m
     double  sigmaVelEast;   // m/s
     double  sigmaVelNorth;  // m/s
     double  sigmaVelUp;     // m/s
     double  sigmaPitch; // degree
     double  sigmaRoll;  // degree
     double  sigmaYaw;   // degree

     double  accX;  // m/s^2
     double  accY;  // m/s^2
     double  accZ;  // m/s^2
     double  gyroX;  // degree/s
     double  gyroY;  // degree/s
     double  gyroZ;  // degree/s
     // 组合导航状态; 0: 初始化; bit0 =1 : INS;  bit1 = 1 : GPS;
     // bit2 = 1 : MC; bit3 = 1 : ODE; bit4 = 1 : Lidar; bit5 = 1 : Camera;
     char  navState;
     //GPS状态 	0: SINGLE_POINT；1： DGPS；2： SBAS；3：PPP； 4： RTK_COM；5: RTK_FIX; 6: OTHER;
     char  gpsState;
     bool  newNavOutFlag;

     char  hardwareFaultInfo; // bit0= 1 : config parameter invalid; bit1 = 1, imu; bit2:gps
     char  inputParamFaultInfo;
     char  alignFaultInfo;
     char  sensorOutLostInfo;
     char  alignState;// tbd
     char  fault2;// tbd

 };

typedef vector<MSFLoutput> MSFLoutputS;

#endif
namespace PublishData {

 struct PbTrajectory : planning::PbTrajectory
 {

 };

} //end namespace CommonStruct





#endif // STRUCT_H
