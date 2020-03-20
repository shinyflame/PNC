#include<iostream>
#include<vector>
#include<math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <iomanip>
#include<string>

#include "hdmap_util.h"
#include "../common/pnc_information.h"



using namespace std;
using Eigen::MatrixXd;

namespace hdmap{

//static variate initialization
HDMap* HDMapUtil::base_map_ = nullptr;

//const HDMap* HDMapUtil::BaseMapPtr() { return base_map_.get();}

const HDMap* HDMapUtil::BaseMapPtr() { return const_cast<HDMap*>(base_map_);}


MapPoint HDMapUtil::getOriginalPoint(){

   return base_map_->original_point;

}



bool HDMapUtil::loadMapFromFile(const std::string &map_filename){
      FILE *file = nullptr;
      //FILE *file;
      file = fopen(map_filename.c_str(),"r");
      //file = fopen((char*)map_filename.data(),"r");
  //  file = fopen((char*)map_filename.c_str(),"r");
    //file = fopen(map_filename.data(),"r");
    if(file == nullptr)
    {

        cout << "Open Map file failed" << endl;
        return false;
    }

    cout << "Start reading map file. " << endl;

    /** declare variables **/
    int num_lanes;
    int num_points;
    int num_lightgroups;
    int num_lightunits;
    int num_lightunit_ids;
    int index_lane;
    int index_point;
    int int_value;
    double temp_check_number;
    int num_Front;
    int num_Behind;
    int num_Charging;
    int num_Recycling;
    int num_polygon_vertices;
    double double_value;
    double check_number;
    char version_num[8];
    std::string current_version;
    LaneType type;
    LaneShape shape;
    LaneTurn turn;
    BoundaryType Boundary_type;
    LightType lighttype;
    SiteType  sitetype;

    // initialize base_map_
    base_map_ = new HDMap();



    /** read and store statistics into variables **/
    if(!feof(file))
    {
        // get value
        fread(&int_value, sizeof(int), 1, file);
        base_map_->id = int_value;
        //cout << "base_map_id::  " << base_map_->id <<endl;
        fread(&version_num, sizeof(char), 8, file);
        current_version=version_num;

        //fread(&int_value, sizeof(double), 1, file);
        // assign value to version number

        for(int i = 0; i < 8;i++){
        base_map_->version[i] = version_num[i];
        }
        //cout << "V: " <<  base_map_->version << endl;

        //base_map_->version_num = double_value;
        // get values and assign values to orignial point
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.point_enu.x = double_value;
        //cout<<"original_point.point_enu.x="<<double_value<<endl;
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.point_enu.y = double_value;
        //cout<<"original_point.point_enu.y="<<double_value<<endl;
        base_map_->original_point.point_enu.z = 0.;
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.point_llh.lon = double_value;
        //cout<<"original_point.point_llh.lon="<<double_value<<endl;
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.point_llh.lat = double_value;
        //cout<<"original_point.point_llh.lat="<<double_value<<endl;
        base_map_->original_point.point_llh.height = 0.;
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.euler_angles.yaw = double_value;
       // cout<<"original_point.euler_angles.yaw="<<double_value<<endl;
        base_map_->original_point.euler_angles.pitch = 0.;
        base_map_->original_point.euler_angles.roll = 0.;
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.kappa = double_value;
         //cout<<"original_point.kappa="<<double_value<<endl;
        fread(&double_value,sizeof(double),1,file);
        base_map_->original_point.s = double_value;
        //cout<<"original_point.s="<<double_value<<endl;

        // num of lanes
        fread(&num_lanes, sizeof(int), 1, file);
        base_map_->lanes_num = num_lanes;

        fread(&int_value, sizeof(int), 1, file);
        base_map_->num_inner_lanes = int_value;
        //cout<<"lanes_num="<<num_lanes<<endl;
        // lanes -- allocate memory
        vector<Lane> lanes(num_lanes);
        if(base_map_->lanes_num == 0){
            cout <<"Read map failed"<<endl;
            return false;
        }

        // for every lane, assign values to its attributes
        for(index_lane = 0; index_lane < num_lanes; index_lane++)
        {

            // lane ids
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].id = int_value;
            //cout<<"lanes[index_lane].id="<<int_value<<endl;
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].left_lane_id = int_value;
            //cout<<"lanes[index_lane].left_lane_id="<<int_value<<endl;
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].right_lane_id = int_value;
            //cout<<"lanes[index_lane].right_lane_id="<<int_value<<endl;

            fread(&num_Front,sizeof(int),1,file);
            lanes[index_lane].num_front_lanes = num_Front;
            if(num_Front > 0){
               for(int i = 0; i < num_Front; i++)
               {
                   fread(&int_value,sizeof(int),1,file);
                   lanes[index_lane].front_lane_ids.push_back(int_value);
               }
            }

            fread(&num_Behind,sizeof(int),1,file);
            lanes[index_lane].num_behind_lanes = num_Behind;
            if(num_Behind > 0){
               for(int i = 0; i < num_Behind;i++)
               {
                   fread(&int_value,sizeof(int),1,file);
                   lanes[index_lane].behind_lane_ids.push_back(int_value);
               }
            }

            fread(&double_value,sizeof(double),1,file);
            lanes[index_lane].cost = double_value;
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].side_slip = int_value;
            //cout << index_lane+1 <<":" <<lanes[index_lane].side_slip  <<endl;
            // lane type, shape, turn type
            fread(&type,sizeof(LaneType),1,file);
            lanes[index_lane].type = type;
            //cout<<"lanes[index_lane].type="<<lanes[index_lane].type<<endl;
            fread(&shape,sizeof(LaneShape),1,file);
            lanes[index_lane].shape = shape;
            //cout<<"lanes[index_lane].shape="<<lanes[index_lane].shape<<endl;
            fread(&turn,sizeof(LaneTurn),1,file);
            lanes[index_lane].turn_type = turn;


            // speed limit
            fread(&double_value,sizeof(double),1,file);
            lanes[index_lane].speed_limit = double_value;
            // traffic light info
            fread(&num_lightgroups,sizeof(int),1,file);
            lanes[index_lane].traffic_light_info.num_light_groups = num_lightgroups;

            if(num_lightgroups != 0){
               vector<TrafficLightGroup> Light_groups(num_lightgroups);
               for (int i = 0;i < num_lightgroups;i++){
                   fread(&int_value,sizeof(int),1,file);
                   Light_groups[i].id = int_value;
                   fread(&int_value,sizeof(int),1,file);
                   Light_groups[i].num_lights = int_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.point_3D.x = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.point_3D.y = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.point_3D.z = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.point_llh.lon = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.point_llh.lat = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.point_llh.height = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.euler_angles.roll = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.euler_angles.pitch = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   Light_groups[i].position.euler_angles.yaw = double_value;

                   fread(&num_lightunits,sizeof(int),1,file);
                   Light_groups[i].num_light_units = num_lightunits;
                   vector<TrafficLightUnit> Light_units(num_lightunits);
                   for(int j = 0; j < num_lightunits;j++){
                       fread(&int_value,sizeof(int),1,file);
                       Light_units[j].id = int_value;
                       fread(&int_value,sizeof(int),1,file);
                       Light_units[j].num_lights = int_value;
                       fread(&lighttype,sizeof(LightType),1,file);
                       Light_units[j].light_type = lighttype;
                   }
                   Light_groups[i].light_units = Light_units;
               }
               lanes[index_lane].traffic_light_info.light_groups = Light_groups;

             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].traffic_light_info.start_s = double_value;
             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].traffic_light_info.end_s = double_value;
            }

            fread(&num_lightunit_ids,sizeof(int),1,file);
            lanes[index_lane].num_light_unit_ids = num_lightunit_ids;
            if (num_lightunit_ids !=0){
                vector<int> lightunit_ids(num_lightunit_ids);
                for(int m = 0; m < num_lightunit_ids;m++){
                    fread(&int_value,sizeof(int),1,file);
                    lightunit_ids[m] = int_value;
                }
                lanes[index_lane].light_unit_ids = lightunit_ids;
            }

            // station
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].station.id = int_value;
//            cout<<"station.id"<<lanes[index_lane].station.id<<endl;
            if(int_value != -1){
             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].station.stop_duration_allowed = double_value;
             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].station.start_s = double_value;
             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].station.end_s = double_value;
            }

            // Terminal
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].terminal.id = int_value;
//            cout<<"station.id"<<lanes[index_lane].station.id<<endl;
            if(int_value != -1){
             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].terminal.start_s = double_value;
             fread(&double_value,sizeof(double),1,file);
             lanes[index_lane].terminal.end_s = double_value;
            }




            // num of central points
            fread(&num_points,sizeof(int),1,file);
            // allocate memory for central_points
            lanes[index_lane].central_points_num = num_points;
            cout<< "lane id: " << index_lane + 1 << endl;
            cout<<"lanes[index_lane].central_points_num:"<<lanes[index_lane].central_points_num<<endl;
            vector<MapPoint> points(num_points);
            if(lanes[index_lane].central_points_num == 0){
                cout <<"Read map failed"<<endl;
                return false;
            }

           // for every point on this lane, assign values to its attributes
            for(index_point = 0; index_point < num_points; index_point++)
            {
                fread(&double_value,sizeof(double),1,file);
                points[index_point].point_enu.x = double_value;
                fread(&double_value,sizeof(double),1,file);
                points[index_point].point_enu.y = double_value;
                points[index_point].point_enu.z = 0.;
                fread(&double_value,sizeof(double),1,file);
                points[index_point].point_llh.lon = double_value;
                fread(&double_value,sizeof(double),1,file);
                points[index_point].point_llh.lat = double_value;
                points[index_point].point_llh.height = 0.;
                fread(&double_value,sizeof(double),1,file);
                points[index_point].euler_angles.yaw = double_value;
                //cout <<"yaw::" << points[index_point].euler_angles.yaw <<endl;
                if(points[index_point].euler_angles.yaw > 180 || points[index_point].euler_angles.yaw < (-1)*180){
                    cout <<"Read map failed"<<endl;
                    return false;
                }

                points[index_point].euler_angles.pitch = 0.;
                points[index_point].euler_angles.roll = 0.;
                fread(&double_value,sizeof(double),1,file);
                points[index_point].kappa = double_value;
                fread(&double_value,sizeof(double),1,file);
                points[index_point].s = double_value;

                // central_point
                if (index_point == num_points/2)
                    lanes[index_lane].central_point = points[index_point];  // this can be changed by the  geometrical centre

            } // end for loop - for each central point
             lanes[index_lane].central_points = points;
//             if (index_lane == 21)
//                cout << index_lane <<endl;
            // left_boundary
            fread(&Boundary_type,sizeof(BoundaryType),1,file);
            lanes[index_lane].left_boundary.type = Boundary_type;
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].left_boundary.point_num = int_value;
            // for loop - left boundary points
            vector<Point3D> left_boundary_points(lanes[index_lane].left_boundary.point_num);
            if(lanes[index_lane].left_boundary.point_num == 0){
                cout <<"Read map failed"<<endl;
                return false;
            }
            for(int j = 0; j < lanes[index_lane].left_boundary.point_num; j++){
                fread(&left_boundary_points[j].x,sizeof(double),1,file);
                fread(&left_boundary_points[j].y,sizeof(double),1,file);
                fread(&left_boundary_points[j].z,sizeof(double),1,file);
            }
            lanes[index_lane].left_boundary.points = left_boundary_points;

            // right_boundaryBoundaryType
            fread(&Boundary_type,sizeof(BoundaryType),1,file);
            lanes[index_lane].right_boundary.type = Boundary_type;
            //cout<<lanes[index_lane].right_boundary.type<<"lanes[index_lane].right_boundary.type="<<endl;
            fread(&int_value,sizeof(int),1,file);
            lanes[index_lane].right_boundary.point_num = int_value;
            // for loop - right boundary points
            vector<Point3D> right_boundary_points(lanes[index_lane].right_boundary.point_num);
            if(lanes[index_lane].right_boundary.point_num == 0){
                cout <<"Read map failed"<<endl;
                return false;
            }
            for(int j = 0; j < lanes[index_lane].right_boundary.point_num; j++){
                fread(&right_boundary_points[j].x,sizeof(double),1,file);
                fread(&right_boundary_points[j].y,sizeof(double),1,file);
                fread(&right_boundary_points[j].z,sizeof(double),1,file);
            }
            lanes[index_lane].right_boundary.points = right_boundary_points;

        } // end for loop

        base_map_->lanes = lanes;
        // ChargingSites
               fread(&num_Charging,sizeof(int),1,file);
               if(num_Charging != 0){
               vector<Site> ChargingSites(num_Charging);
               for(int i = 0; i < num_Charging;i++){
                   fread(&int_value,sizeof(int),1,file);
                   ChargingSites[i].id = int_value;
                   fread(&sitetype,sizeof(SiteType),1,file);
                   ChargingSites[i].type = sitetype;
                   fread(&double_value,sizeof(double),1,file);
                   ChargingSites[i].position.x = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   ChargingSites[i].position.y = double_value;
                   fread(&double_value,sizeof(double),1,file);
                   ChargingSites[i].position.z = double_value;
                   fread(&num_polygon_vertices,sizeof(int),1,file);
                   vector<Point3D> polygonvertices(num_polygon_vertices);
                   for(int j = 0; j < num_polygon_vertices; j++)
                   {
                       fread(&double_value,sizeof(double),1,file);
                       polygonvertices[j].x = double_value;
                       fread(&double_value,sizeof(double),1,file);
                       polygonvertices[j].y = double_value;
                       fread(&double_value,sizeof(double),1,file);
                       polygonvertices[j].z = double_value;
                   }
                   ChargingSites[i].polygon_vertices = polygonvertices;
               }
               base_map_->charging_sites = ChargingSites;}


               // RecyclingSites
               fread(&num_Recycling,sizeof(int),1,file);
               if (num_Recycling != 0){
               vector<Site> RecyclingSites(num_Recycling);
               for(int i = 0; i < num_Recycling;i++){
                   fread(&int_value,sizeof(int_value),1,file);
                   RecyclingSites[i].id = int_value;
                   fread(&sitetype,sizeof(SiteType),1,file);
                   RecyclingSites[i].type = sitetype;
                   fread(&double_value,sizeof(double_value),1,file);
                   RecyclingSites[i].position.x = double_value;
                   fread(&double_value,sizeof(double_value),1,file);
                   RecyclingSites[i].position.y = double_value;
                   fread(&double_value,sizeof(double_value),1,file);
                   RecyclingSites[i].position.z = double_value;
                   fread(&num_polygon_vertices,sizeof(int_value),1,file);
                   vector<Point3D> polygonvertices(num_polygon_vertices);
                   for(int j = 0; j < num_polygon_vertices; j++)
                   {
                       fread(&double_value,sizeof(double),1,file);
                       polygonvertices[j].x = double_value;
                       fread(&double_value,sizeof(double),1,file);
                       polygonvertices[j].y = double_value;
                       fread(&double_value,sizeof(double),1,file);
                       polygonvertices[j].z = double_value;
                   }
                   RecyclingSites[i].polygon_vertices = polygonvertices;
               }
               base_map_->recycling_sites = RecyclingSites;}


               for(int i = 0; i < 4;i++){
                   fread(&double_value,sizeof(double_value),1,file);
                   base_map_->range_x_y.push_back(double_value);
                   //cout << "range_xy:  "<<base_map_->range_x_y[i] <<endl;
               }
               for(int j = 0; j < 4;j++){
                   fread(&double_value,sizeof(double_value),1,file);
                   base_map_->range_lon_lat.push_back(double_value);
                   //cout << "range_LL:  "<<base_map_->range_lon_lat[j] <<endl;
               }

//              fread(&double_value,sizeof(double_value),1,file);
              fread(&int_value,sizeof(int_value),1,file);
              if(int_value<0 || int_value>100){
                  fclose(file);
                  cout <<"Read map successfully"<<endl;
                  return true;
              }else{
              base_map_->start_map_id = int_value;
              cout<<"start map id is "<<int_value<<endl;
              fread(&int_value,sizeof(int_value),1,file);
              base_map_->end_map_id = int_value;
              cout<<"end map id is "<<int_value<<endl;

             //add one number to check map for reading
             fread(&check_number,sizeof(double),1,file);
               }
    } // end if

    fclose(file);

    //check map for reading
    if(check_number != 2019.73){
        cout <<"Read map failed"<<endl;
        return false;
    }
    else{
        cout <<"Read map successfully"<<endl;
        return true;
    }
}



Point3D HDMapUtil::mapCoordintateTransform(PointLLH &vehicle_global_coordinates){

    Point3D vehicle_map_coordinates;

    PointLLH anchor_point;
    // get the LLH of anchor_point, which is the first point of the map - original point.
    anchor_point = base_map_->original_point.point_llh;
    anchor_point.lon = anchor_point.lon * M_PI / 180;
    anchor_point.lat = anchor_point.lat * M_PI / 180;
    anchor_point.height = anchor_point.height;


    /// YOU NEED TO CHANGE THE UNIT OF LLH FROM DEGREE INTO RADIAN
    /// BEFORE YOU DO THE CALCULATIONS BELLOW
    /// THE UNIT OF (LON, LAT) ARE STORED AS DEGREE ANYWHERE, WHILE IN c++, RADIAN IS USED TO CALCULATE "trigonometric function"
    vehicle_global_coordinates.lon = vehicle_global_coordinates.lon * M_PI / 180;
    vehicle_global_coordinates.lat = vehicle_global_coordinates.lat * M_PI / 180;
    vehicle_global_coordinates.height = vehicle_global_coordinates.height;

    // assign values
    double f = 1 / 298.257;
    double e = sqrt(2*f - f*f);
    double a = 6378137.;
    double N0 = a / sqrt(1 - e*e*sin(anchor_point.lat)*sin(anchor_point.lat));
    double N1 = a / sqrt(1 - e*e*sin(vehicle_global_coordinates.lat)*sin(vehicle_global_coordinates.lat));

    // calculate the coordinate of the traffic light in map coordination system
    MatrixXd R(3,3);
    MatrixXd M(3,1);
    MatrixXd X(3,1);

    R(0,0) = -sin(anchor_point.lon);
    R(0,1) = cos(anchor_point.lon);
    R(0,2) = 0;
    R(1,0) = -sin(anchor_point.lat) * cos(anchor_point.lon);
    R(1,1) = -sin(anchor_point.lat) * sin(anchor_point.lon);
    R(1,2) = cos(anchor_point.lat);
    R(2,0) = cos(anchor_point.lat) * cos(anchor_point.lon);
    R(2,1) = cos(anchor_point.lat) * sin(anchor_point.lon);
    R(2,2) = sin(anchor_point.lat);

    M(0,0) = (N1+vehicle_global_coordinates.height) * cos(vehicle_global_coordinates.lat) * cos(vehicle_global_coordinates.lon)
            - (N0+anchor_point.height) * cos(anchor_point.lat) * cos(anchor_point.lon);
    M(1,0) = (N1+vehicle_global_coordinates.height) * cos(vehicle_global_coordinates.lat) * sin(vehicle_global_coordinates.lon)
            - (N0+anchor_point.height) * cos(anchor_point.lat) * sin(anchor_point.lon);
    M(2,0) = (N1*(1-e*e) + vehicle_global_coordinates.height) * sin(vehicle_global_coordinates.lat)
            - (N0*(1-e*e) + anchor_point.height) * sin(anchor_point.lat);

    X = R*M;

    vehicle_map_coordinates.x = X(0,0);
    vehicle_map_coordinates.y = X(1,0);
    vehicle_map_coordinates.z = X(2,0);

    return vehicle_map_coordinates;
}



}// end namespace

