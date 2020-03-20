#ifndef HDMAPPUBLISH_H
#define HDMAPPUBLISH_H
#include <vector>
#include <list>
#include <string>

using namespace std;


//////////////////////////////namespace PublishData//////////////////////////////////

namespace PublishData {


enum LaneType {
    NONE = 1,
    CITY_DRIVING = 2,
    BIKING = 3,
    SIDEWALK = 4,
    PARKING = 5,
    SHOULDER = 6
};
enum LaneShape{
     STRAIGHT = 1,
     CIRCLE = 2,
     S_CURVE = 3,
     U_CURVE = 4

    };
enum LaneTurn {
    NO_TURN = 1,
    LEFT_TURN = 2,
    RIGHT_TURN = 3,
    U_TURN = 4
};

enum BoundaryType {
   DOTTED_YELLOW = 1, // can change lanes
   DOTTED_WHITE = 2,  // can change lanes
   SOLID_YELLOW = 3,  // can change lanes
   SOLID_WHITE = 4,   // can't change lanes
   DOUBLE_YELLOW = 5, // can't change lanes
   CURB = 6           // can't change lanes
 };


// for Traffic Light  -- more to be added
enum LightType{
    ARROW_STRAIGHT = 1,
    ARROW_LEFT = 2,
    ARROW_RIGHT = 3,
    ARROW_STRAIGHT_LEFT = 4,
    CIRCLE_STRAIGHT = 5,
    CIRCLE_LEFT = 6,
    CIRCLE_RIGHT = 7,
    CIRCLE_STRAIGHT_LEFT = 8
};


enum SiteType{
    Charging = 1,
    Recycling = 2
};


struct Point3D{
    double x;
    double y;
    double z;
};


struct PointLLH{
    double lon;
    double lat;
    double height;
};

// Roll/pitch/yaw that represents a rotation with intrinsic sequence z-x-y.
// in world coordinate (East/North/Up)
// The roll, in (-pi/2, pi/2), corresponds to a rotation around the y-axis.
// The pitch, in [-pi, pi), corresponds to a rotation around the x-axis.
// The yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
// The direction of rotation follows the right-hand rule.
struct EulerAngles{
    double pitch;
    double roll;
    double yaw;
};


struct MapPoint{
    Point3D  point_enu;
    PointLLH point_llh;
    EulerAngles euler_angles;
    double s;
    double kappa;
};


struct Width{
    double s;
    double center_to_left;
    double center_to_right;
};


struct Position{
    Point3D point_3D;
    PointLLH point_llh;
    EulerAngles euler_angles;
};


struct TrafficLightUnit{
    int id;
    int num_lights;
    LightType light_type;
};


struct TrafficLightGroup{
    int id;
    int num_lights;
    Position position;
    vector<double> light_frame_attributes = {2.8, 0.95, 0.8}; // The default value is national standard value; unit: metre.
    int num_light_units;
    vector<TrafficLightUnit> light_units;
};

// the structure stored in hdmap
struct TrafficLightInfo{
    int num_light_groups;
    vector<TrafficLightGroup> light_groups;
    double  start_s;
    double  end_s;
};


// the structure transmitted to Vision Module
struct TrafficLightGroups{
    PointLLH map_original_point;
    vector<TrafficLightGroup> light_groups;
};


struct Station{
    int      id;   // -1 meaning no station
    double   stop_duration_allowed;    // second (s)
    double   start_s;
    double   end_s;
};

struct Terminal{
    int      id;   // -1 meaning no terminal
    double   start_s;
    double   end_s;
};

struct Destination{
    bool has_destination;  // false meaning no desination on the route
    double start_s;   // the projected shift of position on the route
    double end_s;
    Point3D position;
};

struct Site{
    int id;
    SiteType type;
    Point3D position;
    vector<Point3D> polygon_vertices;
};


struct Boundary{
    BoundaryType type;
    int point_num;
    vector<Point3D> points;
};

} // namespace PublishData


////////////////In namespace hdmap//////////////////

namespace hdmap{

// the structure stored in route (for Planning Module)
struct TrafficLight{
    bool has_traffic_light;
    vector<int> ids;    // it stores ids of the TrafficLightUnit.
    double  start_s;
    double  end_s;
};

}
/////////////////////////////////////////////////////


////////////////////////// namespace PublishData ////////////////////////
namespace PublishData{

struct Instruction
{
    bool select_again=false;
    bool use_instruction=false;   /// whether to  use instruction to select map or not
    int selected_scene_id=0;    ///  selected scene id
    int selected_map_id=0;
    bool switch_map=false;

};

struct Response{
    bool current_map_cleaned=false;
    bool current_scene_cleaned=false;

};

struct Lane{
     int id;
     int   left_lane_id;     //if (id = -1) the id don't exsit;
     int  right_lane_id;
     int  num_front_lanes;
     vector<int> front_lane_ids;
     int num_behind_lanes;
     vector<int> behind_lane_ids;

     double cost;  // length of the lane
     bool side_slip;

     LaneType  type;
     LaneShape shape;
     LaneTurn  turn_type;
     double   speed_limit;
     TrafficLightInfo traffic_light_info;   // This will be stored in upper layer, say roadSegment, so it's not repetitively stored in lanes in parallel.(when standard hdmap format is made)
     int num_light_unit_ids;
     vector<int> light_unit_ids;    // it stores ids of the TrafficLightUnit that functions on the lane.
     Station     station;
     Terminal terminal;   // THIS IS A NEW ONE

     struct MapPoint central_point;
     int central_points_num;
     vector<struct MapPoint> central_points;
     Boundary left_boundary;
     Boundary right_boundary;
};


struct HDMap {
    int id;
    char version[8];
    MapPoint original_point;
    int      lanes_num;
    int      num_inner_lanes;   // added on 2-019/11/25, for zhangqiu DEFAULT GLOBAL PATH
    vector<Lane> lanes;
    vector<Site> charging_sites;
    vector<Site> recycling_sites;

    vector<double> range_x_y;  // in the order of Xmin, Xmax, Ymin, Ymax
    vector<double> range_lon_lat;  // in the order of LONmin, LONmax, LATmin, LATmax

    int start_map_id;
    int end_map_id;
};


struct Route{
    int id;
    vector<int>         lanes_id;
    vector<MapPoint>    reference_points;
    vector<Width>       widths;
//    vector<Width>       road_widths;
    hdmap::TrafficLight        traffic_light;
    double   speed_limit;
    LaneTurn turn_type;
    Station  station;
    Terminal terminal;
    Destination destination;
};

struct PncRoute{
    bool side_slip = false;
    bool on_route;
    int is_update = 0;   // updated on 20101014
    MapPoint map_original_point;
    Route route;
};

// output type for Planning Module
typedef vector<PncRoute> PncRoutes;

struct Curve_Coefficient
{
  double A[6];
  double B[6];
};


} // end namespace PublishData

//////////////////////////////////////////////////////////////////////////


/////////////////////////namespace hdmap////////////////////////////

namespace hdmap{

// for MULTIPLE_MAP_SELF_ACCOMODATED
enum MapAssignment{
    MAP_ASSIGNED = 1,
    MAP_UNASSIGNED = 2
};


// config parameters for map module
struct ConfigParam {
    double LOOK_FORWARD_DISTANCE;
    double LOOK_BACKWARD_DISTANCE;
    double STEP_SIZE;
    double MAX_DISTANCE;
    double UPDATE_DISTANCE;
    double UPDATE_DISTANCE_BACK;
    double MAX_NUMBER;
    bool DEBUG_STATE;
    string OVERLAP_MAP_LANE_MATCHES;
};


struct ReferenLineWithParam
{
  //vector<ENUPoint> reference_point;//llh + xyzv +euler_anglue
  vector<PublishData::MapPoint> start_point; // xyz theta k dk ddk s
  vector<PublishData::Curve_Coefficient> curve_param; // a[6] b[6]

  vector<int> lane_ids;//lane id
  //vector<PathPoint> discretized_reference_point;//xyz theta k dk ddk s
  PublishData::MapPoint original_point;//llh + xyzv +euler_anglue
};


struct ParkingSpace
{
  PublishData::Point3D center_point;
  double heading;
  double length;
  double width;

};


////////////////////// typedefs /////////////////////////////

typedef PublishData::LaneType LaneType;
typedef PublishData::LaneShape LaneShape;
typedef PublishData::LaneTurn LaneTurn;
typedef PublishData::BoundaryType BoundaryType;
typedef PublishData::LightType LightType;
typedef PublishData::SiteType SiteType;
typedef PublishData::Point3D Point3D;
typedef PublishData::PointLLH PointLLH;
typedef PublishData::EulerAngles EulerAngles;
typedef PublishData::MapPoint MapPoint;
typedef PublishData::Width Width;
typedef PublishData::Position Position;
typedef PublishData::TrafficLightUnit TrafficLightUnit;
typedef PublishData::TrafficLightGroup TrafficLightGroup;
typedef PublishData::TrafficLightInfo TrafficLightInfo;
typedef PublishData::TrafficLightGroups TrafficLightGroups;
typedef PublishData::Station Station;
typedef PublishData::Terminal Terminal;
typedef PublishData::Destination Destination;
typedef PublishData::Site Site;
typedef PublishData::Boundary Boundary;
typedef PublishData::Lane Lane;
typedef PublishData::HDMap HDMap;
typedef PublishData::Route Route;
typedef PublishData::PncRoute PncRoute;
typedef PublishData::PncRoutes PncRoutes;
typedef PublishData::Curve_Coefficient Curve_Coefficient;
typedef PublishData::Instruction Instruction;
typedef PublishData::Response Response;

////////////////////////////////////////// typedefs /////////


}//end namespace hdMap



#endif // HDMAPPUBLISH_H

