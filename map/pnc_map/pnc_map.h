#ifndef PNCMAP_H
#define PNCMAP_H

#include <vector>
#include <list>
#include <map>

#include "../HdMapPublish.h"
#include "../common/pnc_information.h"
#include "../common/pnc_util.h"

namespace hdmap {

class PncMap{

    public:

       /**
         * @brief PncMap constructor
         * @param base_map
         */
        PncMap(const HDMap *base_map, ConfigParam config_param);

        /**
          * @brief PncMap destructor
          */
         ~PncMap();

        /**
         * @brief GetRoutes: get a list of routes that's egligable for vehicle to drive on
         * @param vehicle_state
         * @param pnc_routes: the one we will fill in as the result
         * @param global_path: a list of lane_ids, part of the input from Routing Module
         * @return
         */
        bool GetRoutesWithRouting(const Point3D &vehicle_state, Point3D &destination, vector<int> &global_path,
                                  int id, double s, PncRoutes &pnc_routes);
        bool GetHistoryStatus()const {return hasHistory_;}
        void SetHistoryStatus(bool flag) {hasHistory_=flag;}

    private:

        // hdMap being used
        const HDMap *base_map_;

        // config param
        ConfigParam config_param_;

        // for updating routes in hidstory use
        bool hasHistory_ = false;
        int historyID_ = 1;
        list<PncRoute> *routes_;
        map<int, vector<double>> *shifts_;
        map<int, bool> near_terminal_;
        map<int, bool> near_destination_;
        int route_id_max_ = 0;    // to keep track of the maximum route_id so far. Use it as "++route_id_max_".
        int num_neighbor_lanes_;   // as a flag indicating whether to delete routes or not
        bool ifCheckOverlap_ = true;
        bool ifCheckOnRoute_ = true;
        bool division_part_ = false;
        bool division_segment_ = false;
        vector<int> global_path_;
        Point3D destination_;

        int is_update_ = 0;
        double pre_vehicle_s_=0;
        double cur_vehicle_s_=0;


        /**
         * @brief updateRouteProperties
         * @param id
         * @param s
         * @param route_id
         * @param route
         */
        void updateRouteProperties(int id, double s, Route &route);

        /**
         * @brief getValidRoutes
         * @param routes
         */
        void getValidRoutes(Point3D vehicle_state, int id, list<PncRoute> *const routes);

        /**
         * @brief setGlobalPath
         * @param id
         * @param destination
         * @param global_path
         */
        bool setGlobalPath(int id, Point3D &destination, vector<int> &global_path);

        /**
         * @brief disToLane
         * @param vehicle_state
         * @param lane_id
         * @return
         */
        double disToLane(Point3D vehicle_state, int lane_id);

        double disToRoute(Point3D vehicle_state, vector<MapPoint> &route_points);

        /**
         * @brief checkOverlap
         * @param id
         * @param isOverlaped
         * @param overlap_lanes
         */
        void checkOverlap(int id, bool &isOverlaped, vector<int> &overlap_lanes);

        /**
         * @brief containedIn
         * @param element
         * @param list_elements
         * @return
         */
        bool containedIn(int element, vector<int> list_elements);

        /**
         * @brief getNeighborLaneIDs
         * @param id
         * @param neighbor_lane_ids
         */
        void getNeighborLaneIDs(int id, vector<int> &neighbor_lane_ids);

        /**
         * @brief deleteRoutes
         * @param id
         * @param routes
         */
        void deleteRoutes(int id,  list<PncRoute> * const routes);

        /**
         * @brief checkIsOnRoute
         * @param id
         * @param lanes_id
         * @param is_on_segment
         */
        void checkIsOnRoute(int id, vector<int> lanes_id, bool &is_on_segment);

        /**
         * @brief calculateNeighbors
         * @param id
         * @param num_neighbor_lanes
         */
        void calculateNeighbors(int id, int &num_neighbor_lanes);

        /**
         * @brief getTraversables
         * @param global_path
         * @param traversable_lanes
         */
        void getTraversable(const vector<int> &global_path, vector<vector<int>> &traversable_lanes);

        /**
         * @brief modifyRoute
         * @param route_id
         * @param is_on_segment
         * @param routes
         */
        void modifyRoute(int route_id, bool is_on_segment, int is_update, list<PncRoute> * const routes);

        /**
         * @brief deleteRoute
         * @param route_id
         * @param routes
         */
        void deleteRoute(int route_id, list<PncRoute> * const routes);

        /**
         * @brief finalUpdate
         * @param this_route
         * @param p_shift
         * @param lane_id
         * @param route_id
         * @param is_on_segment
         * @param destination
         * @param traversable_lanes
         * @param routes
         */
        void finalUpdate(const Route &this_route, double p_shift, int lane_id, int route_id, bool is_on_segment, int is_update, Point3D destination,
                                 const vector<vector<int>> &traversable_lanes, list<PncRoute> * const routes);


        bool updateRouteBackWards(const Route &this_route, double p_shift, int lane_id, int route_id,
                                          bool is_on_segment, int is_update, list<PncRoute> * const routes);

        /**
         * @brief updateRoutes
         * @param this_route
         * @param p_shift
         * @param lane_id
         * @param route_id
         * @param is_on_segment
         * @param traversable_lanes  to be assigned
         * @param routes  to be assigned
         */
        void updateRoutes(const Route &this_route, double p_shift, int lane_id, int route_id, bool is_on_segment, int is_update, Point3D destination,
                                  const vector<vector<int>> &traversable_lanes, list<PncRoute> * const routes);


        /**
         * @brief createRoutes -- overloaded for input from global planning
         * @param lane_ids
         * @param projectedShifts
         * @param id
         * @param traversable_lanes
         * @param routes
         */
        bool createRoutes(const vector<int> &lane_ids, const vector<double> &projectedShifts, int id, Point3D destination,
                          const vector<vector<int>> &traversable_lanes, list<PncRoute> *const routes);




        /**
         * @brief checkNeighborLanes
         * @param id
         * @param lane_ids
         * @param projectedPoints
         * @param vehicle_state
         */
        bool checkNeighborLanes(int id, vector<int> &lane_ids, vector<double> &projectedShifts, Point3D vehicle_state);


        /**
         * @brief generatePncRoute
         * @param lane_id
         * @param points
         * @param lane_IDs
         * @param s
         * @param shifts
         * @param vehicle_lane_id
         * @param route_id
         * @param has_on_route
         * @param routes
         */
        void generatePncRoute(int lane_id, vector<MapPoint> &points, const vector<int> &lane_IDs, double s,
                              vector<double> &shifts, int vehicle_lane_id, int route_id,
                              Point3D destination, const vector<int> &destination_lanes,
                              bool &has_on_route, list<PncRoute> * const routes);

        /**
         * @brief extendPoints
         * @param lastID
         * @param lane_IDs
         * @param shifts
         * @param points
         */
        void extendPoints(int lastID, const vector<int> &lane_IDs,
                          const vector<double> &shifts, vector<MapPoint> &points);

        void extendPointsBackwards(int frontID, const vector<int> &lane_IDs,
                                           const vector<double> &shifts, vector<MapPoint> &points);

        /**
         * @brief swapRoutes
         * @param routes
         */
        void swapRoutes(list<PncRoute> *routes);


        /**
         * @brief substituteRouteSegment
         * @param new_routeSeg
         * @param routes
         * @param route_id
         */
        void substituteRoute(PncRoute new_routeSeg, list<PncRoute> *const routes, int route_id);

        /**
         * @brief getNearestPoint
         * @param point
         * @param points
         * @param nearest_index
         */
        void getNearestPoint(Point3D point, const vector<MapPoint> points, int &nearest_index);


        /**
         * @brief getShiftOnRoute
         * @param vehicle_state
         * @param route
         * @param vs_shift
         * @param behind_index
         */
        bool getShiftOnRoute(const Point3D &vehicle_state, const Route &route, double &vs_shift, int &behind_index);


        /**
         * @brief getShiftOnLane
         * @param route_id
         * @param vs_shift
         * @param p1_s
         * @param behind_index
         * @param lane_id
         * @param p_shift
         */
        void getShiftOnLane(const vector<int> &laneIDs, int route_id, double vs_shift, double p1_s, int behind_index,
                                    int &lane_id, double &p_shift);


        /**
         * @brief change information such as station and traffic light of current route
         * @param id
         * @param points
         * @param lane_IDs
         * @param s
         */
        Route changeRouteInfo(int id, const vector<int> &lane_IDs,
                             double s, Route old_route);


        /**
         * @brief getStation
         * @param last_ID
         * @param points
         * @param shifts
         * @param route
         */
        void getStation(int id, int index, double s, const vector<double> &shifts, Route &route);

        /**
         * @brief getTerminal
         * @param id
         * @param s
         * @param last_ID
         * @param points
         * @param shifts
         * @param route
         */
        void getTerminal(int id, int index, double s, const vector<double> &shifts, Route &route);


        /**
         * @brief getOverlapZone
         * @param last_ID
         * @param points
         * @param shifts
         * @param route
         */
        void getTrafficLight(int id, int index, double s, const vector<double> &shifts, Route &route);


        /**
         * @brief getWidth
         * @param initial_lane_index
         * @param route
         */
        void getWidth(vector<double> &shifts, Route &route);

        void SetRoadWidth(list<PncRoute>& routes);
        /**
         * @brief getDestination
         * @param lane_IDs
         * @param destination_lanes
         * @param destination
         * @param route
         */
        void getDestination(const vector<int> &lane_IDs, const vector<int> &destination_lanes, Point3D destination, Route &route);


        /**
         * @brief generate a route from the information supplied by the arguments
         * @param id  lane id
         * @param points   points calculated before for the route to be generated
         * @param lane_IDs  lane ids calculated before for the route to be generated
         * @param p  projected point of the vehicle on the lane
         * @param shifts  the shift of the points
         * @return route generated in this function
         */
        Route generateRoute(int id, vector<MapPoint> &points, const vector<int> &lane_IDs, double s,
                                           vector<double> &shifts, Point3D destination, const vector<int> &destination_lanes);


        Route generateRouteTmp(int id, vector<MapPoint> &points, const vector<int> &lane_IDs,
                                           double s, vector<double> &shifts);

        /**
         * @brief checkLeftLanes
         * @param lane_id
         * @param laneID_pointIndex
         * @param lane_ids
         * @param projectedPoints
         */
        void checkLeftLanes(int lane_id, vector<int> &lane_ids, vector<double> &projectedShifts, Point3D vehicle_state);


        /**
         * @brief checkRightLanes
         * @param lane_id
         * @param lane_ids
         * @param projectedPoints
         */
        void checkRightLanes(int lane_id, vector<int> &lane_ids, vector<double> &projectedShifts, Point3D vehicle_state);


        /**
         * @brief getShifts (overloaded for using result of global planning as input)
         * @param p -- the mapPoint whose shift on its corresponding lane of the hdmap (to obtained)
         * @param id -- the id of the corresponding lane for the mapPoint
         * @param shifts -- the list of shifts for the route aimed by p (to obtained)
         * @param lane_IDs -- the list of IDs of the lanes which are overlaped by the route aimed by p
         */
        bool getShifts(double p_s, int id, const vector<vector<int>> &traversable_lanes, vector<vector<double>> &list_shifts,
                       vector<vector<int> > &list_lane_IDs, bool &near_destination);


        bool findStepIndex(int id, const vector<vector<int>> &traversable_lanes, int &step_index);

        bool getIntersectionList(int id, double current_forward_s, double current_length, const vector<vector<int>> &traversable_lanes,
                                 int step_index, int &count, vector<vector<int>> &list_intersection, int &break_index);

        void calIntersection(const vector<int> &first_list, const vector<int> &second_list, vector<int> &intersection);

        bool haveIntersection(const vector<int> &first_list, const vector<int> &second_list);

        /**
         * @brief finalExtendShifts
         * @param route_id
         * @param shifts
         * @param lane_IDs
         * @param points
         */
        void finalExtendShifts(int route_id, vector<double> &shifts, vector<int> &lane_IDs, vector<MapPoint> &points);


        bool extendShiftsBackwards(vector<double> &shifts, vector<int> &lane_IDs);

        /**
         * @brief extendShifts
         * @param shifts
         * @param lane_IDs
         * @param traversable_lanes
         * @param list_shifts
         * @param list_lane_IDs
         * @param near_terminal
         * @param near_destination
         * @return
         */
        bool extendShifts(const vector<double> &shifts, const vector<int> &lane_IDs,
                          const vector<vector<int>> &traversable_lanes, vector<vector<double>> &list_shifts,
                          vector<vector<int>> &list_lane_IDs, bool &near_terminal, bool &near_destination);


        /**
         * @brief getNeighborPoints in central line
         * @param nearest_index
         * @param p1
         * @param p2
         */
        void getNeighborPoints(int laneIndex, int nearest_index, MapPoint &p1, MapPoint &p2);


        /**
         * @brief getLeftFrontPointIndex
         * @param pointIndex
         * @param laneIndex
         * @return
         */
        void getLeftFrontPointIndex(int pointIndex, int laneIndex, vector<int> &lanes_id, int &frontLaneIndex, int &frontPointIndex);

        /**
         * @brief getRightFrontPointIndex
         * @param pointIndex
         * @param laneIndex
         * @return
         */
        void getRightFrontPointIndex(int pointIndex, int laneIndex, vector<int> &lanes_id, int &frontLaneIndex, int &frontPointIndex);


        /**
         * @brief get a map point according to its shift on the lane
         *        when there's no sampled point corresponding to this shift stored
         * @param s  shift of the point on this lane
         * @param points
         * @return map point corresponding to the shift
         */
        MapPoint getPointFromShift(double s, int laneIndex);


        /**
         * @brief getLeftPointFromShift
         * @param s
         * @param laneIndex
         * @return
         */
        MapPoint getLeftPointFromShift(double s, int laneIndex);


        /**
         * @brief getRightPointFromShift
         * @param s
         * @param laneIndex
         * @return
         */
        MapPoint getRightPointFromShift(double s, int laneIndex);



        /**
         * @brief getPoint3DFromShift
         * @param s
         * @param laneIndex
         * @return
         */
        Point3D getPoint3DFromShift(double s, int laneIndex);


        /**
         * @brief calculate the distance between an object of Point3D type and MapPoint type
         * @param target
         * @param p  map point
         * @return the distance between vehicle position and a mapPoint
         */
        double calculateDistance(const Point3D &target, MapPoint p);


        /**
         * @brief calculate the distance between a point of Point3D type and MapPoint type
         * @param postion  the position of the traffic light
         * @param p  map point
         * @return the distance between vehicle position and a mapPoint
         */
        double calDistance(Point3D position, MapPoint p);

        double calDistance(Point3D position, Point3D p);

        /**
         * @brief calculate the distance between two points of MapPoint type
         * @param position
         * @param p1 p2
         * @return distance between two mapPoints
         */
        double calDis(MapPoint &p1, MapPoint &p2);


        /**
         * @brief calDisToLine -- not in use
         * @param p  the point outside line
         * @param p1 one of the points on line
         * @param p2 another one of the points on line
         * @return
         */
        double calDisToLine(Point3D p, MapPoint p1, MapPoint p2);

        double calDisToLine(Point3D p, Point3D p1, Point3D p2);

        double calDisToLine(MapPoint p, Point3D p1, Point3D p2);

        /**
         * @brief getNearestReferencePoints
         * @param k
         * @param position
         * @param a
         * @return
         */
        vector<int> getNearestReferencePoints(Point3D position, const vector<MapPoint> &a);



        /**
         * @brief getProjectedShift
         * @param laneIndex
         * @param nearest_index
         * @param p1
         * @param p2
         * @param vs
         * @return  a pair where the first value stands for the lane id the projected vehicle point is in,
         *         the second stands for the projected shift
         */
        pair<int, double> getProjectedShift(int laneIndex, int nearest_index, MapPoint p1, MapPoint p2, Point3D vs);


        /**
         * @brief binary search to get the index of the nearest point in a
         * @param low
         * @param high
         * @param key
         * @param a
         * @return the index of the nearst point found in a
         */
        int binarySearch(int low, int high, double key, const vector<MapPoint> &a);


        /**
         * @brief interpolateUsingLinearApproximation
         * @param p1
         * @param p2
         * @param s
         * @param laneIndex
         * @return
         */
        MapPoint interpolateUsingLinearApproximation(MapPoint &p1, MapPoint &p2, double s, int laneIndex);


        /**
         * @brief interpolate3D
         * @param p1
         * @param p2
         * @param s
         * @return
         */
        Point3D interpolate3D(MapPoint &p1, MapPoint &p2, double s, int index);


        /**
         * @brief getShiftByInterpolation
         * @param p1
         * @param p2
         * @param location
         * @return the shift of the interpolated point
         */
        double getShiftByInterpolation(MapPoint p1, MapPoint p2, Point3D location);


};


} // end namespace

#endif // PNCMAP_H
