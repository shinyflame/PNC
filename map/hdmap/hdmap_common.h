#ifndef HDMAP_COMMON_H
#define HDMAP_COMMON_H

#include "../HdMapPublish.h"
#include "../common/pnc_information.h"

namespace  hdmap {


class HDMapCommon{

public:

    /**
     * @brief HDMapCommon
     * @param base_map
     */
    HDMapCommon(const HDMap *base_map, ConfigParam config_param): base_map_(base_map), config_param_(config_param),
        has_history_(false), history_laneID_(1), deviated(false){}


    /**
     * @brief getProjection
     * @param id_point
     * @param vehicle_state
     * @param id
     * @param p
     */
    bool getProjection(const Point3D &vehicle_state, double yaw_vehicle, int &id, double &s);


    /**
     * @brief getProjectionOnLane
     * @param pair_p
     * @param vehicle_state
     */
    bool getProjectionOnLane(const Point3D &vehicle_state, int &v_id, double &v_shift);


    /**
     * @brief CheckIsotonicity
     * @param yaw_vehicle
     * @param yaw_map_point
     * @return
     */
    bool CheckIsotonicity(double yaw_vehicle, double yaw_map_point);

    /**
     * @brief getProjectionOnLane
     * @param pair_p
     */
    bool getFirstProjectionOnLane(const Point3D &vehicle_state, double yaw_vehicle, int &v_id, double &v_shift);


    void getProjectionWithNearestPoint(const Point3D &vehicle_state, int nearest_lane_index, int nearest_point_index, int &id, double &s);


    bool getNearestPoint(const Point3D &vehicle_map_coordinates, double &distance, int &nearest_lane_index, int &nearest_point_index);

    /**
     * @brief find the nearest lanes for the vehicle (linear search)
     * @param k  the number of nearest lanes to be found
     * @param vs  vehicle state
     * @param a  vector of central points on a lane
     * @return a vector of lane ids
     */
    bool getNearestLanes(int k, const vector<Lane> &lanes, const Point3D &vs, vector<int> &nearest_lane_ids);


    /**
     * @brief calculate the distance between an object of Point3D type and MapPoint type
     * @param target
     * @param p  map point
     * @return the distance between vehicle position and a mapPoint
     */
    double calculateDistance(const Point3D &target, MapPoint p);


    /**
     * @brief compare function (overwrites the function in std for function 'sort')
     * @param a first element
     * @param b second element
     * @return true if the first is smaller than the second in the way it's designed to compare; otherwise false
     */
    static bool comp(const pair<int,double> a, const pair<int,double> b);


    /**
     * @brief getNeighborPoints in central line
     * @param nearest_index
     * @param p1
     * @param p2
     */
    void getNeighborPoints(int laneIndex, int nearest_index, MapPoint &p1, MapPoint &p2);


    /**
     * @brief getProjectedShift
     * @param laneIndex
     * @param nearest_index
     * @param p1
     * @param p2
     * @param vs
     * @param id
     * @param s
     */
    void getProjectedShift(int laneIndex, int nearest_index, MapPoint p1, MapPoint p2, Point3D vs,
                           int &id, double &s);


    /**
     * @brief getTrafficLightGroups
     * @param id
     * @param s
     * @param traffic_light_groups
     * @return
     */
    bool getTrafficLightGroups(int id, double s, TrafficLightGroups &traffic_light_groups);

    /**
     * @brief getTrafficLightInfo
     * @param lane_id
     * @param s
     * @param traffic_light_info
     * @return
     */
    bool getTrafficLightInfo(int lane_id, double s, TrafficLightInfo &traffic_light_info);

    // flag of vehicle deviating from drivable area
    bool deviated;

    // flag of first time entering drivable area


    // hdMap being used
    const HDMap *base_map_;

    bool GetHistoryStatus()const {return has_history_;}
    void SetHistoryStatus(bool flag) {has_history_=flag;}


private:

    bool has_history_;
    // config param
    ConfigParam config_param_;

    // for updating use
    int history_laneID_;

};


} // end namespace

#endif // HDMAP_COMMON_H
