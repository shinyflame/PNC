#include <math.h>
#include <algorithm>
#include <iostream>

#include "hdmap_common.h"


namespace hdmap{


bool HDMapCommon::getProjection(const Point3D &vehicle_state, double yaw_vehicle,int &id, double &s){

    if (has_history_){
        if (!getProjectionOnLane(vehicle_state, id, s)){
            deviated = true;
            return false;
        }
    }
    else{
        if (!getFirstProjectionOnLane(vehicle_state, yaw_vehicle, id, s)){
            deviated = true;
            return false;
        }
    }

    history_laneID_ = id;

    if (s < 0){
        cout << "The projected shift of vehicle on lane is smaller than 0, "
                "it seems like the input position of the vehicle is too far away from the lanes/map points. " << endl;
        return false;
    }         // DOUBLE CHECK THIS, NOT SURE IF THIS IS NEEDED JUST IN SIMULATION OR REAL-WORLD TEST...

    return true;

}



bool HDMapCommon::getProjectionOnLane(const Point3D &vehicle_state, int &v_id, double &v_shift){

        /// 1. get the projection of vehicle position to the lane
        int id, index;
        double distance, minDistance = config_param_.MAX_NUMBER; /*, difHeading, minDifHeading;*/
        vector<int> nearest_lane_ids;

        // 1.1 set "nearest_lane_ids" manually
        if (base_map_->lanes[history_laneID_-1].left_lane_id != -1)
            nearest_lane_ids.push_back(base_map_->lanes[history_laneID_-1].left_lane_id);  // left
        if (base_map_->lanes[history_laneID_-1].right_lane_id != -1)
            nearest_lane_ids.push_back(base_map_->lanes[history_laneID_-1].right_lane_id);  // right
        //... back: we don't talk about parking/heading off for now
        for (int front_lane_id: base_map_->lanes[history_laneID_-1].front_lane_ids)
            nearest_lane_ids.push_back(front_lane_id);                                  // front
        // extra to be added (the front lane ids of "history_lane's behind lane)
        if (base_map_->lanes[history_laneID_-1].num_behind_lanes > 0){
            for (int lane_id: base_map_->lanes[base_map_->lanes[history_laneID_-1].behind_lane_ids[0] - 1].front_lane_ids)
                nearest_lane_ids.push_back(lane_id);
        }
        else
            nearest_lane_ids.push_back(history_laneID_); // itself

        // 1.2 get the nearest lane ids and corresponding point indices for vehicle state
        // by calculating all distances between the vehicle position and the points in the neighbor lanes respectively:
        for (int i = 0; i < nearest_lane_ids.size(); i++){
            for (int j = 0; j < base_map_->lanes[nearest_lane_ids[i] - 1].central_points_num; j++){
                distance = calculateDistance(vehicle_state, base_map_->lanes[nearest_lane_ids[i] - 1].central_points[j]);
                if (distance < minDistance){
                    minDistance = distance;
                    id = nearest_lane_ids[i];
                    index = j;
                }
            }
        }

        if (minDistance > 6){
            cout << "Latter projectopn ,Vehicle not in drivable area of self-driving mode. "
                    "Manual navigation towards drivable area needs to be done. "<< endl;
            return false;
        }

        /// 1.2 get the projected point for the vehicle
        MapPoint p1;
        MapPoint p2;
        // 1.2.1 get the nearest two points to the vehicle on lane
        getNeighborPoints(id - 1, index, p1, p2);
        // 1.2.2 get projected point
        getProjectedShift(id - 1, index, p1, p2, vehicle_state, v_id, v_shift);

        return true;

}


bool HDMapCommon::CheckIsotonicity(double yaw_vehicle, double yaw_map_point){

    cout << "Input yaw_vehicle: " << yaw_vehicle << endl;
    cout << "Input yaw_map_point: " << yaw_map_point << endl;

    // tranform yaw_map_point(+180, -180) into the same scale of yaw_double(0,360)
    if (yaw_vehicle >= 0 && yaw_vehicle <= 180)
        yaw_vehicle = -yaw_vehicle;
    else
        yaw_vehicle = -(yaw_vehicle - 360);

    cout << "trasformed yaw_vehicle: " << yaw_vehicle << endl;

    // check

    if (fabs(yaw_vehicle - yaw_map_point) <= 45)
        return true;
    else if (yaw_vehicle * yaw_map_point < 0 && fabs(yaw_vehicle - yaw_map_point) > 180){
        if (yaw_vehicle < 0){
            yaw_vehicle = yaw_vehicle + 360;
            if (fabs(yaw_vehicle - yaw_map_point) <= 45)
                return true;
        }
        else {
            yaw_map_point = yaw_map_point + 360;
            if (fabs(yaw_vehicle - yaw_map_point) <= 45)
                return true;
        }
    }

    return false;
}




bool HDMapCommon::getFirstProjectionOnLane(const Point3D &vehicle_state, double yaw_vehicle, int &v_id, double &v_shift){

        /// 1. get the projection of vehicle position to the lane
        /// 1.1 get the nearest lane ids and corresponding point indices for vehicle state
        int id, index = -1;
        double distance, minDistance;
        vector<int> nearest_lane_ids;
        double k_value =10;

        cout<<"getFirstProjectionOnLane begin"<<endl;

        if (base_map_->lanes_num < 10)
            k_value = base_map_->lanes_num;

        // get the nearest lanes by linear search (here, we want to search for 2 nearest lanes):
        if (!getNearestLanes(k_value, base_map_->lanes, vehicle_state, nearest_lane_ids))
            return false;

        // calculate all distances between the vehicle position and the points in the nearest lanes respectively:
        minDistance = config_param_.MAX_NUMBER;
        cout<<"hdmap_common Isptonicity check"<<endl;
        for (int i = 0; i < k_value; i++){
            for (int j = 0; j < base_map_->lanes[nearest_lane_ids[i] - 1].central_points_num; j++){
                distance = calculateDistance(vehicle_state, base_map_->lanes[nearest_lane_ids[i] - 1].central_points[j]);              
                if (distance < minDistance && CheckIsotonicity(yaw_vehicle, base_map_->lanes[nearest_lane_ids[i] - 1].central_points[j].euler_angles.yaw)){
                    minDistance = distance;
                    id = nearest_lane_ids[i];
                    index = j;
                }
            }
        }

        if (index == -1 || minDistance > 3.5){
             PncInformation::Instance()->MutableGetPncStatus()->projection_status=PncInformation::ProjectionStatus::PROJECTION_DISTANCE_OUT_OF_RANGE;
            cout << "First projection, Vehicle not in drivable area of self-driving mode. "
                    "Manual navigation towards drivable area needs to be done. "<< endl;
            return false;
        }

        has_history_ = true;   //  YOU SHOULD ONLY SET THIS WHEN IT'S A VALID FIRST PROJECTION,
                                    // SO THAT HISTORY_ID IS ASSIGNED FOR NEXT USE

        /// 1.2 get the projected point for the vehicle
        /// 1.2.1 get the nearest two points to the vehicle on lane
        MapPoint p1;
        MapPoint p2;

        getNeighborPoints(id - 1, index, p1, p2);
        getProjectedShift(id - 1, index, p1, p2, vehicle_state, v_id, v_shift);
        cout<<"getFirstProjectionOnLane end"<<endl;
        return true;

}


void HDMapCommon::getProjectionWithNearestPoint(const Point3D &vehicle_state,int nearest_lane_index, int nearest_point_index, int &id, double &s){
    /// get the projected point for the vehicle
    /// get the nearest two points to the vehicle on lane
    MapPoint p1;
    MapPoint p2;

    getNeighborPoints(nearest_lane_index, nearest_point_index, p1, p2);
    getProjectedShift(nearest_lane_index, nearest_point_index, p1, p2, vehicle_state, id, s);
}



bool HDMapCommon::getNearestPoint(const Point3D &vehicle_state, double &minDistance, int &nearest_lane_index, int &nearest_point_index){
    /// 1. get the projection of vehicle position to the lane
    /// 1.1 get the nearest lane ids and corresponding point indices for vehicle state
    cout<<"inside getNearestPoint"<<endl;
    int id, index;
    double distance;
    vector<int> nearest_lane_ids;
    double k_value = 10;

    if (base_map_->lanes_num < 10)
        k_value = base_map_->lanes_num;

    // get the nearest lanes by linear search (here, we want to search for 2 nearest lanes):
    if (!getNearestLanes(k_value, base_map_->lanes, vehicle_state, nearest_lane_ids))
        return false;

    // calculate all distances between the vehicle position and the points in the nearest lanes respectively:
    minDistance = config_param_.MAX_NUMBER;
    for (int i = 0; i < k_value; i++){
        for (int j = 0; j < base_map_->lanes[nearest_lane_ids[i] - 1].central_points_num; j++){
            distance = calculateDistance(vehicle_state, base_map_->lanes[nearest_lane_ids[i] - 1].central_points[j]);
            if (distance < minDistance){
                minDistance = distance;
                id = nearest_lane_ids[i];
                index = j;
            }
        }
    }

    if (minDistance > 6){
        PncInformation::Instance()->MutableGetPncStatus()->projection_status=PncInformation::ProjectionStatus::PROJECTION_DISTANCE_OUT_OF_RANGE;
        cout<<"projection min distance out of range"<<endl;
        return false;
    }
    nearest_lane_index = id - 1;
    nearest_point_index = index;

    return true;
}


bool HDMapCommon::getNearestLanes(int k, const vector<Lane> &lanes, const Point3D &vs,  vector<int> &nearest_lane_ids){   // use const reference   // use different function name

    if (lanes.size() <= 0 || lanes.size() > 200){  // any more conditions?
        cout << "In Function getNearestLanes of Class PncMap: Length of argument lanes invalid! " << endl;
        return false;
    }

    // declare nearest lanes
    vector<pair<int,double>> nearest_lanes;  // first -- lane id; second -- distance

    // assign the first k elements in a to nearest_lanes
    for (int i = 0; i < k ; i++)
        nearest_lanes.push_back(pair<int,double>(i+1, calculateDistance(vs, lanes[i].central_point)));

    // sort the vector
    sort(nearest_lanes.begin(), nearest_lanes.end(), comp);

    int tag;
    pair<int,double> currentLane;
    // for each other element(lane) in a(lanes)
    for (int i = k; i < lanes.size(); i++){
        tag = k;
        currentLane.first = i+1;
        currentLane.second =  calculateDistance(vs, lanes[i].central_point);
       // for each element in vector 'distances' (go from the last one - greatest), check the position where currentPoint should be inserted
        for (int j = k-1; j >= 0; j--){
            if (currentLane.second >= nearest_lanes[j].second)
                break;
            else
                tag = j;
        }
        // overwrite backwards from tag'th to k'th element
        for (int j = k-1; j > tag; j--)
            nearest_lanes[j] = nearest_lanes[j-1];
        if (tag < k)
            nearest_lanes[tag] = currentLane;
    }

    // get the ids
    for (int i = 0; i < k ; i++)
        nearest_lane_ids.emplace_back(nearest_lanes[i].first);

    return true;
}


double HDMapCommon::calculateDistance(const Point3D &target, MapPoint p){

    return sqrt((target.x - p.point_enu.x)*(target.x - p.point_enu.x) + (target.y - p.point_enu.y)*(target.y - p.point_enu.y));
}


bool HDMapCommon::comp(const pair<int,double> a, const pair<int,double> b){
    return a.second < b.second;
}


/// THERE'S FRONT_LANE_IDS[0] USED HERE, THINK ABOUT WHETHER AND HOW YOU NEED TO CHANGE IT
void HDMapCommon::getNeighborPoints(int laneIndex, int nearest_index, MapPoint &p1, MapPoint &p2){

    int id;
    int size = base_map_->lanes[laneIndex].central_points_num;

    // get the point before the nearest map point
    if (nearest_index == 0){
        if (base_map_->lanes[laneIndex].num_behind_lanes > 0){
            id = base_map_->lanes[laneIndex].behind_lane_ids[0];
            p1 = base_map_->lanes[id - 1].central_points[base_map_->lanes[id - 1].central_points_num - 2];
        }
        else
            p1 = base_map_->lanes[laneIndex].central_points[nearest_index];
    }
    else
        p1 = base_map_->lanes[laneIndex].central_points[nearest_index - 1];

    // get the point after the nearest map point
    if (nearest_index == size - 1){
        if (base_map_->lanes[laneIndex].num_front_lanes > 0){
            id = base_map_->lanes[laneIndex].front_lane_ids[0];
            p2 = base_map_->lanes[id - 1].central_points[1];
        }
        else
            p2 = base_map_->lanes[laneIndex].central_points[nearest_index];
    }
    else
        p2 = base_map_->lanes[laneIndex].central_points[nearest_index + 1];

}


// by doing vector dot product
void HDMapCommon::getProjectedShift(int laneIndex, int nearest_index, MapPoint p1, MapPoint p2, Point3D vs,
                                    int &id, double &s){

    double s1, s2, length;
    int laneIndex_p1, laneIndex_p;
    int size = base_map_->lanes[laneIndex].central_points_num;

    s1 = p1.s;

    // border case 1
    if(nearest_index == 0){//point index==0

        if (base_map_->lanes[laneIndex].num_behind_lanes <= 0){/////////////question repeated judge
            id = laneIndex + 1;
            s = 0.;
           return;
        }
////////////////////////////////////todo: lack back distance///////////////////////////
        laneIndex_p1 = base_map_->lanes[laneIndex].behind_lane_ids[0] - 1;
        length = base_map_->lanes[laneIndex_p1].central_points[base_map_->lanes[laneIndex_p1].central_points_num - 1].s;
        s2 = p2.s + length;
        s = s1 + ((vs.x - p1.point_enu.x) * (p2.point_enu.x - p1.point_enu.x) + (vs.y - p1.point_enu.y) * (p2.point_enu.y - p1.point_enu.y)) / (s2 - s1);
        if (s > length){
            s -= length;
            laneIndex_p = laneIndex;
        }
        else{
            laneIndex_p = laneIndex_p1;
        }
    }

    // border case 2
    ////////////////////////////////////todo: lack forward distance///////////////////////////
    else if (nearest_index == size - 1){
        // laneIndex_p1 = laneIndex;
        length = base_map_->lanes[laneIndex].central_points[base_map_->lanes[laneIndex].central_points_num - 1].s;
        s2 = p2.s + length;
        s = s1 + ((vs.x - p1.point_enu.x) * (p2.point_enu.x - p1.point_enu.x) + (vs.y - p1.point_enu.y) * (p2.point_enu.y - p1.point_enu.y)) / (s2 - s1);
        if (s > length){
            s -= length;
            laneIndex_p = base_map_->lanes[laneIndex].front_lane_ids[0] - 1;   // doesn't really matter, just use next_lane_ids[0] (in global_lane_ids)
        }
        else
            laneIndex_p = laneIndex;
    }
    else{
        s2 = p2.s;
        s = s1 + ((vs.x - p1.point_enu.x) * (p2.point_enu.x - p1.point_enu.x) + (vs.y - p1.point_enu.y) * (p2.point_enu.y - p1.point_enu.y)) / (s2 - s1);
        laneIndex_p = laneIndex;

    }

    id = laneIndex_p + 1;

}




////////////////////////// For Vision Module ////////////////////////////

// This is the interface between Map and Vision Module
bool HDMapCommon::getTrafficLightGroups(int id, double s, TrafficLightGroups &traffic_light_groups){

    TrafficLightInfo traffic_light_info;

    if (!getTrafficLightInfo(id, s, traffic_light_info))
        return false;

    // add info to traffic_light_groups
    if (traffic_light_info.num_light_groups > 0){
        traffic_light_groups.light_groups = traffic_light_info.light_groups;
    }
    traffic_light_groups.map_original_point = base_map_->original_point.point_llh;

    return true;

}


/// AT THE MOMENT, WE CHECK THE LANE THAT VEHICLE IS ON AND THE FRONT LANE OF THAT
///     CHANGE THIS CONDITION WHENEVER YOU NEED
bool HDMapCommon::getTrafficLightInfo(int lane_id, double s, TrafficLightInfo &traffic_light_info){

    /// YOU NEED TO DO SOMETHING ELSE BEFORE YOU RETURN THE TRAFFIC LIGHT,
    /// BECAUSE WE HAVE CHANGED THE STRUCTURE OF 'TRAFFIC LIGHT'.

    traffic_light_info.num_light_groups = 0;

    // get the nearest front traffic light info
    // 1. check if the traffic light in current lane is in front of the vehicle
    if (base_map_->lanes[lane_id - 1].traffic_light_info.num_light_groups){
        if (s < base_map_->lanes[lane_id - 1].traffic_light_info.end_s){  // this condition can be changed according to the requirements
            //     1.1 if yes, then output this one
            traffic_light_info = base_map_->lanes[lane_id - 1].traffic_light_info;
            return true;
        }
    }


    // 1.2  if not, go on with 2
    // 2. output the traffic light in the front lane
    int front_lane_id = base_map_->lanes[lane_id - 1].front_lane_ids[0];    //// RECHECK THIS WHEN YOU USE "REAL OVERLAP" MAPS
    if (front_lane_id != -1 && base_map_->lanes[front_lane_id - 1].traffic_light_info.num_light_groups)
        traffic_light_info = base_map_->lanes[front_lane_id - 1].traffic_light_info;
    else
        cout << "There's no traffic light in the nearby front. " << endl;

    return true;
}


}
