#include <iostream>
#include <math.h>
#include <algorithm>
#include <list>
#include <map>
#include <utility>
#include "pnc_map.h"


namespace hdmap{

// constructor
PncMap::PncMap(const HDMap *base_map, ConfigParam config_param):base_map_(base_map), config_param_(config_param){
    hasHistory_ = false;
    historyID_ = 1;
    ifCheckOverlap_ = true;
    ifCheckOnRoute_ = true;
    division_part_ = false;
    division_segment_ = false;
    routes_ = new list<PncRoute>;
    shifts_ = new map<int, vector<double>>;
}

// destructor
PncMap::~PncMap(){
    delete routes_;
    delete shifts_;
}

// For maps with overlaps in roads, use global_path from the output of Routing Module
bool PncMap::GetRoutesWithRouting(const Point3D &vehicle_state, Point3D &destination, vector<int> &global_path,
                                  int id, double s, PncRoutes &pnc_routes) {



    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "-------------Inside PncMap-------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;


    cout << "vehicle state: (" << vehicle_state.x << ", " << vehicle_state.y << ")" << endl;
    cout << "id: " << id << endl;
    cout << "s: " << s << endl;


    list<PncRoute> *const routes = new list<PncRoute>;
    vector<vector<int>> traversable_lanes;
    int num_neighbor_lanes = 0;


    ////// CHANGE THE FOLLOWING //////////////////
    //////////////////////////////////////////////////////////
    // 0. set global var: global_path_
    global_path.clear();
    if (!hasHistory_){
        // set default global_path according to lanes_num
        for (int i = 1; i <= base_map_->num_inner_lanes; i++)   // CHAMGE THE RANGE TO BE "base_map_->num_inner_lanes"
            global_path_.push_back(i);
    }
    //////////////////////////////////////////////////////////////


    cout << "Going to set default global path and destination. " << endl;
    if (!setGlobalPath(id, destination, global_path))
        return false;

    if(global_path.size()>0){
        cout<<"current global path is "<<endl;
        for(auto cur_lane:global_path){
            cout<<cur_lane<<" ";
        }
        cout<<endl;
    }

    // 1. get the traversable lanes in the order of global path
    getTraversable(global_path, traversable_lanes);




    // 2. calculate the number of neighbors
    calculateNeighbors(id, num_neighbor_lanes);


    cout << "---- num_neighbor_lanes:  " << num_neighbor_lanes << "-------" << endl;



    /// Check if there's need to update routes (3 cases)
    /// Case 1: first time to call "GetRoutes"
    if (!hasHistory_){
        cout << "Inside Case 1. " << endl;
        hasHistory_ = true;

        // declare variables (for searching other lanes in parallel)
        vector<int> lane_ids;
        vector<double> projectedShifts;

        lane_ids.push_back(id);
        projectedShifts.push_back(s);

//        cout << "checking neighbor lanes. " << endl;

        // check left and right lanes if there are any
        checkNeighborLanes(id, lane_ids, projectedShifts, vehicle_state);

        // create routes and assign to routes_
        if (!createRoutes(lane_ids, projectedShifts, id, destination, traversable_lanes, routes))
            return false;

//        cout << "Getting outside case 1. " <<  endl;
    } // end Case 1


    /// If it's not the first time to call "GetRoutes"
    else{

        // declare variables
        Route route;
        map<int, bool> map_is_on_segment;
        bool is_on_segment;
        double vs_shift=0, p_shift=0;  // shift on route and lane respectively
        int lane_id=0, behind_index=0;

        *routes = *routes_;

        if (num_neighbor_lanes > num_neighbor_lanes_){
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            cout << "division starts." << endl;
            cout << "num_neighbor_lanes: " << num_neighbor_lanes << endl;
            cout << "num_neighbor_lanes_: " << num_neighbor_lanes_ << endl;
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            division_part_ = true;
            division_segment_ = true;
        }

        /// DELETE routes if "multiple lanes converges into less"
        if (num_neighbor_lanes < num_neighbor_lanes_){
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            cout << "Probably just passed an overlap point. " << endl;
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            cout << "-----------------------" << endl;
            deleteRoutes(id, routes);
            ifCheckOverlap_ = true;
            ifCheckOnRoute_ = true;
        }

        *routes_ = *routes;

        if (!division_segment_ && ifCheckOverlap_) {
            bool isOverlaped = false;
            vector<int> overlap_lanes;
            int count = 0;
            int num_routes = 0;

            checkOverlap(id, isOverlaped, overlap_lanes);

            if (isOverlaped) {
                // calculate the distances of vehicle to each lane
                for (PncRoute pnc_route: *routes){
                    num_routes++;
                    cout << "disToRoute: " << disToRoute(vehicle_state, pnc_route.route.reference_points) << endl;
                    if (disToRoute(vehicle_state, pnc_route.route.reference_points) < 0.5)   // set 0.5 to config_param_.MIN_DISTANCE
                        count++;
                }
                if (count == num_routes){ // RECHECK HERE !!!!!!!!!!!!! if there's more complex situations of map
                    cout << "count == num_routes" << endl;
                    ifCheckOverlap_ = false;
                    ifCheckOnRoute_ = false;
                }
            }
        }

        if (division_part_){
            cout << "division_part_ is true. " << endl;

            // calculate the distances of vehicle to each lane
            for (PncRoute pnc_route: *routes){
                if (/*containedIn(id, pnc_route.route.lanes_id) && */ disToRoute(vehicle_state, pnc_route.route.reference_points) > 0.5){   // set 0.5 to config_param_.MIN_DISTANCE
                    division_part_ = false;
                    cout << "found one distance larger than 0.5. division_part_ is set to false." << endl;
                }
            }
        }
 cout<<"before division segment historyID is "<<historyID_<<endl;
        if (division_segment_){
            cout << "division_segment : true. " << endl;
            cout<<"historyID is "<<historyID_<<endl;
            vector<int> division_lanes;
            for (int front_lane_id: base_map_->lanes[historyID_ - 1].front_lane_ids){
                for (PncRoute pnc_route: *routes){
                    if (containedIn(front_lane_id, pnc_route.route.lanes_id))
                        division_lanes.push_back(front_lane_id);
                }
            }

            // calculate front_lane_ids
            vector<int> front_lane_ids;
            for (int lane_id: division_lanes)
                front_lane_ids.push_back(base_map_->lanes[lane_id - 1].front_lane_ids[0]);

            if (containedIn(id, front_lane_ids)){
                division_segment_ = false;
                cout << "division_segment is set to false. " << endl;
            }
            else{
                cout << "division segment is true. " << endl;
                /* check if vehicle is on route by distance from vehicle to route*/
                double min = config_param_.MAX_NUMBER, distance=0;
                int min_route_id;
                for (PncRoute pnc_route: *routes){
                    for (int division_lane_id: division_lanes){
                        if (containedIn(division_lane_id, pnc_route.route.lanes_id)){
                            distance = disToLane(vehicle_state, division_lane_id);
                            if (distance < min){
                                min = distance;
                                min_route_id = pnc_route.route.id;
                            }
                        }
                    }
                }

                cout << "min_route_id: " << min_route_id << endl;

                // assign is_on_segment
                for (PncRoute pnc_route: *routes){
                    if (pnc_route.route.id == min_route_id)
                        map_is_on_segment[min_route_id] = true;
                    else
                        map_is_on_segment[pnc_route.route.id] = false;
                }
            }
        }

        if (ifCheckOnRoute_ && !division_segment_){     //
            // check if the vehicle is on the route for every route in *routes_
            for (PncRoute pnc_route: *routes_){
                route = pnc_route.route;
                is_on_segment = false;
                checkIsOnRoute(id, route.lanes_id, is_on_segment);
                map_is_on_segment[route.id] = is_on_segment;
            }
            // check if there're more than one "is_on_segment" is true
            int flag = 0;
            map<int, bool>::iterator iter;
            for( iter = map_is_on_segment.begin(); iter != map_is_on_segment.end(); ){
                if (iter->second)
                    flag++;
                iter++;
            }
            // reassign map_is_on_segment if needed
            if (flag > 1){
                for (PncRoute pnc_route: *routes_){
                    route = pnc_route.route;
                    is_on_segment = false;
                    if (pnc_route.on_route)
                        is_on_segment = true;
                    map_is_on_segment[route.id] = is_on_segment;
                }
            }
        }

        if (!ifCheckOnRoute_ || division_part_){
            cout << "In case !ifCheckOnRoute_ || division_part_." << endl;
            //assign "map_is_on_segment", where you set each "is_on_segment" to be "is_on_route" of pnc_route on last frame
            for (PncRoute pnc_route: *routes_){
                route = pnc_route.route;
                map_is_on_segment[route.id] = pnc_route.on_route;
            }
        }

        cout << "checking if there's need to update routes properties. " << endl;


        // for each pnc_route, check if there's need to update
        for (PncRoute pnc_route: *routes_){
            route = pnc_route.route;

            if (!getShiftOnRoute(vehicle_state, route, vs_shift, behind_index)){
                cout << "There's something wrong with the configuration of the vehicle movements, " <<
                        "such that the vehicle unexpectedly went further than the length of the route. " << endl;
                return false;
            }

            getShiftOnLane(route.lanes_id, route.id, vs_shift, route.reference_points[behind_index].s, behind_index, lane_id, p_shift);

            cur_vehicle_s_=vs_shift;

            cout << endl << endl << "vs_shift: " << vs_shift << endl;
            cout << "p_shift: " << p_shift << endl << endl;


            // NOW WE SPLIT INTO CASE 2 AND CASE 3
            /// Case 2: no need to update
            if (vs_shift < config_param_.LOOK_BACKWARD_DISTANCE + config_param_.UPDATE_DISTANCE + 0.0001){
                cout << "Inside Case 2. " << endl;
                // -- YOU CAN OPTIMISE BY MODIFYING THE ROUTE INSTEAD OF CREATING A NEW ONE AND SUBSTITUTE IT

                if (vs_shift > config_param_.LOOK_BACKWARD_DISTANCE - config_param_.UPDATE_DISTANCE_BACK - 0.001             
                        ||cur_vehicle_s_>=pre_vehicle_s_){  //// CASE 2.1 -- no update of route points
                    updateRouteProperties(lane_id, p_shift, route);

                    // Substitute the pnc_route stored in routes
                    // recreate new_pnc_route with the new is_on_segment
                    PncRoute new_pnc_route;
                    new_pnc_route.is_update = pnc_route.is_update;
                    new_pnc_route.on_route = map_is_on_segment[route.id];
                    new_pnc_route.map_original_point = base_map_->original_point;
                    new_pnc_route.route = route;
                    new_pnc_route.side_slip = false;
                    if (base_map_->lanes[lane_id - 1].side_slip)
                        new_pnc_route.side_slip = true;

                    // modify the list "routes"(substitute the old one with the new one)
                    substituteRoute(new_pnc_route, routes, route.id);
                }
///////////////////////////////////////////////////////////////////////////////////////////
                else {      //// CASE 2.2 -- BACKING CAR -- UPDATE ROUTE POINTS BACKWARDS

                    cout << endl << "Backing car------Updating route points-----" << endl << endl;

                    ///////////////////////////////////////////////////////////////
                    /// YOU NEED TO ADD EXTRA LOGIC HERE WHEN YOU USE OVERLAP MAPS
                    /// ..................
                    /// THIS IS ULTRA IMPORTANT !!!!
                    ///////////////////////////////////////////////////////////////


                    /// FOR THE MOMENT, YOUR CODE ONLYS NEEDS TO ACOMMODATE:
                    ///     (1) SINGLE/PARALLEL LANES;
                    ///     (2) CONTINUOUS SELF-DRIVING (IN CIRCLE; STOPPED MANUALLY)
                    if (!updateRouteBackWards(route, p_shift, lane_id, route.id, map_is_on_segment[route.id], pnc_route.is_update, routes))
                        return false;

                    cout <<  "succeded in updating backwards." << endl;

                }
////////////////////////////////////////////////////////////////////////////////////////////

            } // end Case 2


            /// Case 3: need to update (exceeding UPDATE_DISTANCE)
            else{
                cout << "Inside Case 3. " << endl;

                if (near_terminal_[route.id] || near_destination_[route.id])   // stop updating from now on
                    // modify the pnc_route with id of route_id in routes
/*add logic for side_slip later*/modifyRoute(route.id, map_is_on_segment[route.id], pnc_route.is_update, routes);
                else
                    updateRoutes(route, p_shift, lane_id, route.id, map_is_on_segment[route.id], pnc_route.is_update,
                            destination, traversable_lanes, routes);
            }  // end Case 3

        } // end for loop (for each route in routes) -- Case 2 & 3

        // assign routes to static variable-routes_
//        SetRoadWidth( *routes);
        pre_vehicle_s_=vs_shift;
        *routes_ = *routes;

    } // end else - if (! hasHistory_)


    cout << "Common bits for case 1, 2, 3. " << endl;

//     SetRoadWidth( *routes);

    // assign historyID_ for next frame
    cout << "ifCheckOverlap_: " << ifCheckOverlap_ << endl;
    cout << "division_segment_: " << division_segment_ << endl;

    if (ifCheckOverlap_ && !division_segment_){
        historyID_ = id;
        cout << "ifCheckOverlap_ && !division_segment_, and historyID_ becomes: " << historyID_ << endl;
    }
    num_neighbor_lanes_ = num_neighbor_lanes;

    cout << "-----------historyID_: " << historyID_ << " -------------" << endl;


    // print all routes:
    int index_route = 0;
    list<PncRoute>::iterator it;
    Route temp_route;
    PncRoute temp_pnc_route;

    for( it = routes->begin(); it != routes->end(); ){
        cout << "no. " << index_route << " route: " << endl;
        cout << "route_id: " << (*it).route.id << endl;
        cout << "on_route: " << (*it).on_route << endl;
//        temp_pnc_route=*it;
        temp_route=(*it).route;//////////////road_widths not accessible
        index_route++;
        cout << "lane ids: " << endl;
        for (int lane_id : (*it).route.lanes_id){
            cout << lane_id <<  " ";
        }
        cout << endl;
        cout << "number of points: "  << (*it).route.reference_points.size() << endl;
//        cout<<"number of road widths points: "<<(*it).route.road_widths.size()<<endl;
        it++;
    }

    cout << "-------------------------------" << endl;
    cout << "-------------------------------" << endl;
    cout << "num of routes: " << index_route << endl;
    cout << "-------------------------------" << endl;
    cout << "-------------------------------" << endl;

    getValidRoutes(vehicle_state, id, routes);    // Perhaps there'll be problems later for other types of map, this operation should be deleted actually.

    cout << "size of routes after getting valid routes: " << routes->size() << endl;


    if (routes->size() > 1){
        cout << "Going to swap routes. " << endl;
        swapRoutes(routes);
    }

    // transform routes into pnc_routes
    list<PncRoute>::iterator itList;
    for( itList = routes->begin(); itList != routes->end(); ){
        pnc_routes.push_back(*itList);
        itList++;
    }
    cout << endl;

    // print all points of the routes:
    cout << "---------------------" << endl;
    cout << "num of routes: " << pnc_routes.size() << endl;
    cout << "---------------------" << endl;
    PncInformation::Instance()->MutableGetPncStatus()->routes_number=pnc_routes.size();
     index_route = 0;
    PncInformation::Instance()->MutableGetPncStatus()->routes_info.clear();
    for (PncRoute pnc_route : pnc_routes){

        PncInformation::RouteInfo current_route_info;
        current_route_info.route_id=pnc_route.route.id;
        current_route_info.side_slip=pnc_route.side_slip;
        current_route_info.points_number=pnc_route.route.reference_points.size();


        cout << "no. " << index_route << " route: " << endl;
        cout << "route_id: " << pnc_route.route.id << endl;
        cout << "side_slip: " << pnc_route.side_slip << endl;

        index_route++;
        cout << "lane ids: " << endl;
        for (int lane_id :pnc_route.route.lanes_id){
            cout << lane_id <<  " ";
           current_route_info.lanes_ids.push_back(lane_id);
        }
        cout << endl;
        cout << "number of points: "  << pnc_route.route.reference_points.size() << endl;
        PncInformation::Instance()->MutableGetPncStatus()->routes_info.push_back(current_route_info);

//        cout << "points: " << endl;
//        for (MapPoint point : pnc_route.route.reference_points){
//            cout << "(" << point.point_enu.x << ", " <<  point.point_enu.y << ") ";
//        }
//        cout << endl;
//        cout << "yaw of the points: " << endl;
//        for (MapPoint point : pnc_route.route.reference_points){
//            cout << point.euler_angles.yaw << " ";
//        }
        cout << endl;
    }

    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "-------------Outside PncMap-------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;
    cout << "--------------------------------" << endl;

    return true;

}


void PncMap::updateRouteProperties(int id, double s, Route &route){

    /// turn_type (choose the one in current lane)
    // [you haven't assign value to this attribute]
    route.turn_type = base_map_->lanes[id - 1].turn_type;

    // find the index of id in lane_IDs
    int index = 0;
    for (int i = 0; i < route.lanes_id.size(); i++){
        if (id == route.lanes_id[i]){
            index = i;
            break;
        }
    }

    /// station (choose the nearest valid station of the lane where the vehicle is in if the vehicle position is in front of that;
    // [ we only need to assign ONE station here ]
    getStation(id, index, s, (*shifts_)[route.id], route);

    // terminal
    getTerminal(id, index, s, (*shifts_)[route.id], route);   // similar to getStation.

    /// traffic light
    getTrafficLight(id, index, s, (*shifts_)[route.id], route);

    /// speed_limit (on_lane)
      route.speed_limit = base_map_->lanes[id - 1].speed_limit;

}


void PncMap::getValidRoutes(Point3D vehicle_state, int id, list<PncRoute> *const routes){

    cout << "-----------------------------" << endl;
    cout << "Inside function getValidRoutes: " << endl;
    cout << "-----------------------------" << endl;

    bool found = false;
    int id1 = -1;
    vector<int> lane_ids;
    Route route;
    double vs_shift, p_shift;
    int behind_index;
    int neareast_index;

    // get list of lanes containing valid lanes
    // (1)
    list<PncRoute>::iterator itList;

    for( itList = routes->begin(); itList != routes->end(); ){
        if((*itList).on_route)
        {
           found = true;
           route = (*itList).route;
           break;
        }
        else
           itList++;
    }

    if (found){
        cout << "Inside case 1... " << endl;

        getShiftOnRoute(vehicle_state, route, vs_shift, behind_index);
        cout<<"after getShiftOnRoute function"<<endl;
        getShiftOnLane(route.lanes_id, route.id, vs_shift, route.reference_points[behind_index].s, behind_index, id1, p_shift);
        cout<<"after getShiftOnLane function"<<endl;

        cout << "id1 is : " << id1 << endl;

        lane_ids.push_back(id1);
        // get left and right lane ids
        if (base_map_->lanes[id1 - 1].left_lane_id != -1)
            lane_ids.push_back(base_map_->lanes[id1 - 1].left_lane_id);
        if (base_map_->lanes[id1 - 1].right_lane_id != -1)
            lane_ids.push_back(base_map_->lanes[id1 - 1].right_lane_id);
    }
    // (2)
    else {
        cout << "Inside case 2... " << endl;
        lane_ids.push_back(id);
        // get left and right lane ids
        if (base_map_->lanes[id - 1].left_lane_id != -1)
            lane_ids.push_back(base_map_->lanes[id - 1].left_lane_id);
        if (base_map_->lanes[id - 1].right_lane_id != -1)
            lane_ids.push_back(base_map_->lanes[id - 1].right_lane_id);
    }

    cout << "-----------------------------" << endl;
    cout << "lane_ids: " << endl;
    for (int lane_id: lane_ids){
        cout << lane_id << " ";
    }
    cout << endl << "-----------------------------" << endl;

    // delete invalid routes
    list<PncRoute>::iterator it;
    for( itList = routes_->begin(); itList != routes_->end(); )
    {
        if(!haveIntersection((*itList).route.lanes_id, lane_ids))
        {
            cout << "going to delete a route. " << endl;
            // delete this route in routes
            for( it = routes->begin(); it != routes->end(); )
            {
                if((*it).route.id == (*itList).route.id)
                {
                   routes->erase(it);
                   break;
                }
                else
                   it++;
            }
        }
        itList++;
    }

    cout << "size of routes: " << routes->size() << endl;

    cout << "-----------------------------" << endl;
    cout << "Outside function getValidRoutes. " << endl;
    cout << "-----------------------------" << endl;

}


bool PncMap::setGlobalPath(int id, Point3D &destination, vector<int> &global_path) {

    // get neighbor lanes
    vector<int> neighbor_lanes;
    getNeighborLaneIDs(id, neighbor_lanes);
    neighbor_lanes.push_back(id);

    // reset global path
    vector<int> v1, v2;
    int index = 0, initial_index = -1;
    for (int lane_id: global_path_){
        if (containedIn(lane_id, neighbor_lanes)){//////////////question why use neighbor lanes
            initial_index = index;
            break;
        }
        index++;
    }

    /// Error Handling for initial_index --- couldn't find id in global_path
    if (initial_index == -1){
        cout << "Error: default global_path was set incorrectly or the lane which vehicle is on is found incorrectly. " << endl;
        return false;
    }

    // copy the elements before id to v1
    copy(global_path_.begin(), global_path_.begin() + initial_index, back_inserter(v1));

    // copy the elements from id to v2
    copy(global_path_.begin() + initial_index, global_path_.end(), back_inserter(v2));

    // set global_path
    copy(v2.begin(), v2.end(), back_inserter(global_path));
    copy(v1.begin(), v1.end(), back_inserter(global_path));

    // set intial destination
    destination = base_map_->lanes[global_path.back() - 1].central_points[base_map_->lanes[global_path.back() - 1].central_points_num - 2].point_enu;

    return true;

}



double PncMap::disToLane(Point3D vehicle_state, int lane_id){

    int nearest_index;
    MapPoint p1, p2;

    getNearestPoint(vehicle_state, base_map_->lanes[lane_id - 1].central_points, nearest_index);

    // get the nearest two points to the vehicle on lane
    getNeighborPoints(lane_id - 1, nearest_index, p1, p2);

    return calDisToLine(vehicle_state, p1, p2);
}


double PncMap::disToRoute(Point3D vehicle_state, vector<MapPoint> &route_points){

    int nearest_index;

    getNearestPoint(vehicle_state, route_points, nearest_index);

    cout << "nearest_index: " << nearest_index << endl;

    return //calDistance(vehicle_state, route_points[nearest_index]);
            calDisToLine(vehicle_state, route_points[nearest_index - 1], route_points[nearest_index + 1]);
}


void PncMap::checkOverlap(int id, bool &isOverlaped, vector<int> &overlap_lanes){

    vector<int> neighbor_lane_ids;
    int count;

    // calculate neighbor_lane_ids -------------- CHECK THE TWO NESTED FOR LOOPS
    getNeighborLaneIDs(id, neighbor_lane_ids);
    neighbor_lane_ids.push_back(id);

    if (neighbor_lane_ids.size() <= 1)
        goto label;
    for (int lane_id1: neighbor_lane_ids){
        count = 0;
        for (int lane_id2: neighbor_lane_ids){
            if (lane_id1 == lane_id2)
                continue;
            if (base_map_->lanes[lane_id1 - 1].num_front_lanes > 0 && base_map_->lanes[lane_id2 - 1].num_front_lanes > 0
                    && base_map_->lanes[lane_id1 - 1].front_lane_ids[0] ==  base_map_->lanes[lane_id2 - 1].front_lane_ids[0])
                count++;
            if (count > 1){
                isOverlaped = true;
                overlap_lanes.push_back(lane_id1);
                overlap_lanes.push_back(lane_id2);
                goto label;
            }
        }
    }

    label:
    ;
}


// if second parameter contains first parameter
bool PncMap::containedIn (int element, vector<int> list_elements){
    for (int ele: list_elements){
        if (ele == element)
            return true;
    }
    return false;
}


void PncMap::getNeighborLaneIDs(int id, vector<int> &neighbor_lane_ids){
    // left
    int current_id = id;
    while (base_map_->lanes[current_id - 1].left_lane_id != -1){
        current_id = base_map_->lanes[current_id - 1].left_lane_id;
        neighbor_lane_ids.push_back(current_id);
    }
    // right
    current_id = id;
    while (base_map_->lanes[current_id - 1].right_lane_id != -1){
        current_id = base_map_->lanes[current_id - 1].right_lane_id;
        neighbor_lane_ids.push_back(current_id);
    }
}


// delete those routes whose lanes_id contain id, but vehicle was not on last time.
void PncMap::deleteRoutes(int id,  list<PncRoute> * const routes){

    cout << "Insdie deleteRoutes. " << endl;

    vector<int> neighbor_lane_ids;

    // calculate neighbor_lane_ids
    getNeighborLaneIDs(historyID_, neighbor_lane_ids);
    cout << "historyID_: " << historyID_ << endl;
    cout << "neighbor lane ids: " << endl;
    for (int lane_id: neighbor_lane_ids) {
        cout << lane_id << " ";
    }
    cout << endl;

    // find them and delete them
    for (PncRoute pnc_route: *routes_){

        cout << "containedIn(id, pnc_route.route.lanes_id): " <<
                containedIn(id, pnc_route.route.lanes_id) << endl;

        cout << "haveIntersection(neighbor_lane_ids, pnc_route.route.lanes_id): "
             << haveIntersection(neighbor_lane_ids, pnc_route.route.lanes_id) << endl;

        if ( containedIn(id, pnc_route.route.lanes_id) &&
                haveIntersection(neighbor_lane_ids, pnc_route.route.lanes_id)) {
            cout << "---------------------------" << endl;
            cout << "---------------------------" << endl;
            cout << "going to delete a route. " << endl;
            cout << "---------------------------" << endl;
            cout << "---------------------------" << endl;

            deleteRoute(pnc_route.route.id, routes);
        }
    }

}


void PncMap::checkIsOnRoute(int id, vector<int> lanes_id, bool &is_on_segment){
    for (int lane_id: lanes_id){
        if (lane_id == id){
            is_on_segment = true;
            break;
        }
    }
}


void PncMap::calculateNeighbors(int id, int &num_neighbor_lanes){
    // left
    int current_id = id;
    while (base_map_->lanes[current_id - 1].left_boundary.type == 2 &&
           base_map_->lanes[current_id - 1].left_lane_id != -1){
        num_neighbor_lanes++;
        current_id = base_map_->lanes[current_id - 1].left_lane_id;
    }
    // right
    current_id = id;
    while (base_map_->lanes[current_id - 1].right_boundary.type == 2 &&
           base_map_->lanes[current_id - 1].right_lane_id != -1){
        num_neighbor_lanes++;
        current_id = base_map_->lanes[current_id - 1].right_lane_id;
    }
}


void PncMap::getTraversable(const vector<int> &global_path, vector<vector<int>> &traversable_lanes){

    vector<int> current_segment;
    int current_lane_id;

    // EXTEND global_path with neighbor lanes
    for (int lane_id: global_path){

        current_lane_id = lane_id;
        current_segment.push_back(lane_id);
        // traverse left
        while (base_map_->lanes[current_lane_id - 1].left_boundary.type == 2){
            current_lane_id = base_map_->lanes[current_lane_id - 1].left_lane_id;
            current_segment.insert(current_segment.begin(), current_lane_id);
        }

        // traverse right
        current_lane_id = lane_id;
        while (base_map_->lanes[current_lane_id - 1].right_boundary.type == 2){
            current_lane_id = base_map_->lanes[current_lane_id - 1].right_lane_id;
            current_segment.push_back(current_lane_id);
        }

        if (traversable_lanes.size() == 0 || traversable_lanes.back() != current_segment)
            traversable_lanes.push_back(current_segment);

        current_segment.clear();
    }
}



void PncMap::modifyRoute(int route_id, bool is_on_segment, int is_update,
                         list<PncRoute> * const routes){
    list<PncRoute>::iterator itList;
    for( itList = routes->begin(); itList != routes->end(); )
    {
        if((*itList).route.id == route_id)
        {
           (*itList).on_route = is_on_segment;
           (*itList).is_update = is_update;
           break;
        }
        else
           itList++;
    }
}


void PncMap::deleteRoute(int route_id, list<PncRoute> * const routes){

    // erase the element whose id is route_id in routes
    list<PncRoute>::iterator itList;
    for( itList = routes->begin(); itList != routes->end(); )
    {
        if((*itList).route.id == route_id)
        {
           routes->erase(itList);
           break;
        }
        else
           itList++;
    }

    // erase the element with key of route_id in shifts_
    map<int, vector<double>>::iterator iter1;
    iter1 = shifts_->find(route_id);
    if (iter1 != shifts_->end())
        shifts_->erase(iter1);
    else
        cout << "Error: In deleteRoute: element not found in shifts_. " << endl;

    // erase the element with key of route_id in near_destination_
    map<int, bool>::iterator iter2;
    iter2 = near_destination_.find(route_id);
    if (iter2 != near_destination_.end())
        near_destination_.erase(iter2);
    else
        cout << "Error: In deleteRoute: element not found in near_destination_. " << endl;

    // erase the element with key of route_id in near_terminal_
    map<int, bool>::iterator iter3;
    iter3 = near_terminal_.find(route_id);
    if (iter3 != near_terminal_.end())
        near_terminal_.erase(iter3);
    else
        cout << "Error: In deleteRoute: element not found in near_terminal_. " << endl;

}



// Similar to updateRoutes()
void PncMap::finalUpdate(const Route &this_route, double p_shift, int lane_id, int route_id, bool is_on_segment,
                         int is_update, Point3D destination, const vector<vector<int>> &traversable_lanes,
                         list<PncRoute> * const routes){

    cout << "Inside function -- finalUpdate. " << endl;

    vector<double> shifts = (*shifts_)[route_id];
    vector<int> lane_IDs = this_route.lanes_id;
    vector<MapPoint> points = this_route.reference_points;

    finalExtendShifts(route_id, shifts, lane_IDs, points);  // -- THIS ONE EXTENDS IN THE LAST LANE, COULD BE WRONG FOR SOME CASE.

    cout << "Just extended shifts. " << endl;

    // generate new route with new points and lane_IDs
    Route route = generateRoute(lane_id, points, lane_IDs, p_shift, shifts, destination, traversable_lanes.back());
    // set id of route
    route.id = route_id;

    cout << "Just generated route. " << endl;

    // create PncRoute
    PncRoute pnc_route;
    pnc_route.is_update = is_update;
    pnc_route.on_route = is_on_segment;  // first one true, the others false
    pnc_route.map_original_point = base_map_->original_point;
    pnc_route.route = route;
    pnc_route.side_slip = false;
    if (base_map_->lanes[lane_id - 1].side_slip)
        pnc_route.side_slip = true;
    /// 7. substitute the route_id-th routeSeg in routes
    substituteRoute(pnc_route, routes, route_id);

    cout << "Getting outside function -- finalUpdate. " << endl;

}


bool PncMap::updateRouteBackWards(const Route &this_route, double p_shift, int lane_id, int route_id,
                                  bool is_on_segment, int is_update, list<PncRoute> * const routes){

    // declare variables:
    vector<double> shifts;
    vector<int> lane_IDs;
    vector<MapPoint> points;
    int frontID;

    /// 4. get the points we need on this passage (with the range of forward length and backward length)
    /// 4.1 leave out the former 5 meter (10 points) and add the latter 10 points into the route (be aware of different cases)
    shifts.clear();
    lane_IDs.clear();
    points.clear();

    // copy points
    points = this_route.reference_points;
    // erase last few points
    points.erase(points.end() - config_param_.UPDATE_DISTANCE_BACK / config_param_.STEP_SIZE, points.end());

    // copy lane IDs
    lane_IDs = this_route.lanes_id;
    frontID = lane_IDs.front();

    // (*) check if there's need to erase the last few lane_IDs
    int j = 0;
    for(int k = (*shifts_)[route_id].size() - config_param_.UPDATE_DISTANCE_BACK / config_param_.STEP_SIZE;
        k < (*shifts_)[route_id].size(); k++){
        if ((*shifts_)[route_id][k-1] > (*shifts_)[route_id][k])
            j++;
    }

    if (j > 0)
        lane_IDs.erase(lane_IDs.end() - j, lane_IDs.end());

    /// 5. get the shifts we need on this passage
    // copy shifts
    shifts = (*shifts_)[route_id];
    // erase shifts
    shifts.erase(shifts.end() - config_param_.UPDATE_DISTANCE_BACK / config_param_.STEP_SIZE, shifts.end());

    // extended "shifts for the points to be added" and "lane_IDs if it's needed" BACKWARDS
    if ( !extendShiftsBackwards(shifts, lane_IDs)){
        cout << "Error: There is no lane behind. " << endl;
        return false;
    }

    // extend points and lane ids
    extendPointsBackwards(frontID, lane_IDs, shifts, points);

    /// 6. generate Route and PncRoute from these points etc.
    // generate route
    Route route = generateRouteTmp(lane_id, points, lane_IDs, p_shift, shifts);  ///// WRITE THIS FUNCTION !!
    // set id of route
    route.id = route_id;
    // create PncRoute
    PncRoute pnc_route;
    pnc_route.is_update = ++is_update_;
    pnc_route.on_route = is_on_segment;  // first one true, the others false
    pnc_route.map_original_point = base_map_->original_point;
    pnc_route.route = route;
    pnc_route.side_slip = false;
    if (base_map_->lanes[lane_id - 1].side_slip)
        pnc_route.side_slip = true;
    /// 7. substitute the route_id-th routeSeg in routes
    substituteRoute(pnc_route, routes, route_id);

    // assign/modify values of static variables
    (*shifts_)[route.id] = shifts;

    return true;
}


void PncMap::updateRoutes(const Route &this_route, double p_shift, int lane_id, int route_id, bool is_on_segment,
                          int is_update, Point3D destination, const vector<vector<int>> &traversable_lanes,
                          list<PncRoute> * const routes){

    // declare variables:
    vector<double> shifts;
    vector<int> lane_IDs;
    vector<MapPoint> points;
    int lastID;
    vector<vector<double>> list_shifts;
    vector<vector<int>> list_lane_IDs;

/////////////////// TEST ///////////////////////
    int count;

    count = 0;
    for (PncRoute pnc_route : *routes){
        count++;
    }
    cout << "count: " << count << endl;
//////////////////////////////////////////////////


    /// 4. get the points we need on this passage (with the range of forward length and backward length)
    /// 4.1 leave out the former 5 meter (10 points) and add the latter 10 points into the route (be aware of different cases)
    shifts.clear();
    lane_IDs.clear();
    points.clear();

    // copy points
    points = this_route.reference_points;
    // erase first few points
    points.erase(points.begin(), points.begin() + config_param_.UPDATE_DISTANCE / config_param_.STEP_SIZE);

    // copy lane IDs
    lane_IDs = this_route.lanes_id;
    lastID = lane_IDs.back();  // index in lane_IDs

    // (*) check if there's need to erase the first few lane_IDs
    int j = 0;
    for(int k = 0; k < config_param_.UPDATE_DISTANCE / config_param_.STEP_SIZE; k++){
        if ((*shifts_)[route_id].at(k+1) < (*shifts_)[route_id].at(k)){
            j++;
        }
    }
    if (j > 0)
        lane_IDs.erase(lane_IDs.begin(), lane_IDs.begin() + j);

    /// 5. get the shifts we need on this passage
    // copy shifts
    shifts = (*shifts_)[route_id];
    // erase shifts
    shifts.erase(shifts.begin(), shifts.begin() + config_param_.UPDATE_DISTANCE / config_param_.STEP_SIZE);

    // extended "shifts for the points to be added" and "lane_IDs if it's needed"
    bool near_terminal = false, near_destination = false;
    if (!extendShifts(shifts, lane_IDs, traversable_lanes, list_shifts, list_lane_IDs, near_terminal, near_destination)){
        if (near_terminal || near_destination){    // 1) If it's near terminal/destination, set flag and next time keep the route unchanged;
            near_terminal_[route_id] = near_terminal;
            near_destination_[route_id] = near_destination;
            cout << endl <<  "going to do final update. " << endl << endl;
            finalUpdate(this_route, p_shift, lane_id, route_id, is_on_segment, is_update, destination, traversable_lanes, routes);
        }
        else{
            // 2) If it's not near terminal/destination, then you should DELETE this route from routes
            cout << "going to delete a route. " << endl;
            deleteRoute(this_route.id, routes);
        }

        cout << "extendShifts returned false. " << endl;

        return;
    }
//    cout << "size of list_shifts: " << list_shifts.size() << endl;

    int list_index = 0;
    vector<MapPoint> this_points;
    vector<int> this_lane_IDs;

    for (vector<double> this_shifts: list_shifts){
        this_lane_IDs = list_lane_IDs[list_index];

        // extend points and lane ids
        this_points = points;
        extendPoints(lastID, this_lane_IDs, this_shifts, this_points);

        /// 6. generate PncRoute from these points
        // generate route
        Route route = generateRoute(lane_id, this_points, this_lane_IDs, p_shift, this_shifts, destination, traversable_lanes.back());

        if (list_index == 0){
            // set id of route
            route.id = route_id;
            // create PncRoute
            PncRoute pnc_route;
            pnc_route.is_update = ++is_update_;
            pnc_route.on_route = is_on_segment;  // first one true, the others false
            pnc_route.map_original_point = base_map_->original_point;
            pnc_route.route = route;
            pnc_route.side_slip = false;
            if (base_map_->lanes[lane_id - 1].side_slip)
                pnc_route.side_slip = true;
            /// 7. substitute the route_id-th routeSeg in routes
            substituteRoute(pnc_route, routes, route_id);
        }
        else{
            // set id of route
            route.id = ++route_id_max_;
            // create PncRoute
            PncRoute pnc_route;
            pnc_route.is_update = ++is_update_;
            pnc_route.on_route = false;
            pnc_route.map_original_point = base_map_->original_point;
            pnc_route.route = route;
            pnc_route.side_slip = false;
            if (base_map_->lanes[lane_id - 1].side_slip)
                pnc_route.side_slip = true;
            // 8. append to the end
            routes->push_back(pnc_route);

            near_destination_[route.id] = false;
            near_terminal_[route.id] = false;
        }

        // assign/modify values of static variables
        (*shifts_)[route.id] = this_shifts;

//        cout << "shifts:  " << endl;
//        for (double shift: this_shifts){
//            cout << shift << " ";
//        }
//        cout << endl;

        list_index++;
    }
}



bool PncMap::createRoutes(const vector<int> &lane_ids, const vector<double> &projectedShifts, int id, Point3D destination,
                          const vector<vector<int>> &traversable_lanes, list<PncRoute> * const routes){

    cout << "Inside createRoutes. " << endl;

    vector<vector<double>> list_shifts;
    vector<vector<int>> list_lane_IDs;
    vector<MapPoint> points;
    int route_id = 0, list_index, projection_index = 0;
    bool has_on_route = false, near_destination;

    for (int lane_id: lane_ids){         //////////////////////order :    left  mid   right
        /// 3.1 get shifts and lane_IDs accroding to the shift of the projected point of vehicle,
        ///     forward length and backward length, and the lanes
        list_shifts.clear();
        list_lane_IDs.clear();
        list_index = 0;

        near_destination = false;
        if (!getShifts(projectedShifts[projection_index], lane_id, traversable_lanes, list_shifts, list_lane_IDs, near_destination))
            return false;

        for (vector<double> shifts: list_shifts){
            points.clear();
            /// 3.2 collect the points according to their shifts
            int j = 0;
            for(int i = 0; i < shifts.size(); i++){
                if (i != 0 && shifts[i] < shifts[i-1])
                    j++;
                points.push_back(getPointFromShift(shifts[i], list_lane_IDs[list_index][j] - 1));
            }

            /// 4. generate PncRoute from those points and append to routes
            generatePncRoute(lane_id, points, list_lane_IDs[list_index], projectedShifts[route_id], shifts, id, route_id,
                             destination, traversable_lanes.back(), has_on_route, routes);

            // assign values to static variables
            (*shifts_)[route_id] = shifts;
            if (near_destination)
                near_destination_[route_id] = true;
            else
                near_destination_[route_id] = false;
            near_terminal_[route_id] = false;

            route_id++;
            list_index++;
        }

        projection_index++;   // UNTESTED
    } // end for loop (for each id/projectedPoint, to generate routes)


    route_id_max_ = --route_id;

    ////Calculate road width
//        SetRoadWidth( *routes);

    /// 5. assign values to other static variables
       *routes_ = *routes;

//    cout << "Exiting createRoutes. " << endl;

    return true;
}


bool PncMap::checkNeighborLanes(int id, vector<int> &lane_ids, vector<double> &projectedShifts, Point3D vehicle_state){

    /// 2.1 WHILE LOOP TO SEARCH MORE "id" & "p" IN LEFT LANES
    /// Method: FIRST get left/right laneID of lane(id); THEN go forward and backward to get the nearest point and lane
    checkLeftLanes(id, lane_ids, projectedShifts, vehicle_state);

    /// 2.2 WHILE LOOP TO SEARCH MORE "id" & "p" IN RIGHT LANES
    checkRightLanes(id, lane_ids, projectedShifts, vehicle_state);

    return true;
}


void PncMap::generatePncRoute(int lane_id, vector<MapPoint> &points, const vector<int> &lane_IDs, double s,
                               vector<double> &shifts, int vehicle_lane_id, int route_id,
                              Point3D destination, const vector<int> &destination_lanes,
                              bool &has_on_route, list<PncRoute> * const routes){

    Route route;
    bool is_on_segment;

    // generate route
    route = generateRoute(lane_id, points, lane_IDs, s, shifts, destination, destination_lanes);

    // set id of route:
    route.id = route_id;
    // check if the vehicle is on the route
    if (lane_id == vehicle_lane_id && !has_on_route){
        is_on_segment = true;
        has_on_route = true;
    }
    else
        is_on_segment = false;

    // create PncRoute
    PncRoute pnc_route;
    pnc_route.is_update = ++is_update_;
    pnc_route.on_route = is_on_segment;
    pnc_route.map_original_point = base_map_->original_point;
    pnc_route.route = route;
    pnc_route.side_slip = false;
    if (base_map_->lanes[lane_id - 1].side_slip)
        pnc_route.side_slip = true;

    // append routeSeg into routes
    routes->push_back(pnc_route);

}



void PncMap::extendPoints(int lastID, const vector<int> &lane_IDs,
                          const vector<double> &shifts, vector<MapPoint> &points){

   // case 1': these points to be added are in the same lane as the last point of former shifts
   if (lastID == lane_IDs.back()){
       // get point from shift respectively in the the lane
       for (int i = (config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE - config_param_.UPDATE_DISTANCE) / config_param_.STEP_SIZE + 1;
            i <= (config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE)/config_param_.STEP_SIZE; i++){
           points.push_back(getPointFromShift(shifts[i], lane_IDs.back() - 1));
       }
   }
   // case 2': these points to be added are in different lanes
   else {
       int k = (config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE - config_param_.UPDATE_DISTANCE) / config_param_.STEP_SIZE;
       int j;
       for (int i = 0; i < lane_IDs.size(); i++){
           if (lane_IDs[i] == lastID)
               j = i;
       }
       if (shifts[k+1] < shifts[k])
           j++;

       // get point from shift respectively in the the lanes
       points.push_back(getPointFromShift(shifts[k+1],lane_IDs[j] - 1));  // the k+1'th point
       for(int i = k+2; i < shifts.size(); i++){  // from the k+2'th point forwards
           if (shifts[i] < shifts[i-1])
               j++;
           points.push_back(getPointFromShift(shifts[i],lane_IDs[j] - 1));
       }
   }
}


void PncMap::extendPointsBackwards(int frontID, const vector<int> &lane_IDs, const vector<double> &shifts, vector<MapPoint> &points){

    if (points.size() < 1)
        cout << "Invalid Argument in Function extendPointsBack of Class PncMap: size of argument 'points' is not greator than 0. " << endl;
    else if (lane_IDs.size() < 1)
        cout << "Invalid Argument in Function extendPointsBack of Class PncMap: size of argument 'lane_IDs' is not greator than 0. " << endl;
    else if (shifts.size() < 1)
        cout << "Invalid Argument in Function extendPointsBack of Class PncMap: size of argument 'shifts' is not greator than 0. " << endl;

    int k = config_param_.UPDATE_DISTANCE_BACK / config_param_.STEP_SIZE - 1;

    int j;
    for (int i = 0; i < lane_IDs.size(); i++){
        if (lane_IDs[i] == frontID)
            j = i;
    }

    // get point from shift respectively in the the lanes
    for(int i = k; i >= 0; i--){
        if (shifts[i] > shifts[i+1])
            j--;
        points.insert(points.begin(), getPointFromShift(shifts[i],lane_IDs[j] - 1));
    }

}


/// There's bug here ! ! !
void PncMap::swapRoutes(list<PncRoute> *routes){

    if (routes->size() < 2)
        cout << "Invalid Argument in Function swapRoutes of Class PncMap: size of argument 'routes' is not greator than 1. " << endl;

    cout << "inside function swapRoutes" << endl;

    // 1. find the laneIndex of laneID_ where the vehicle is on now
    int laneIndex = 0;
    int i = 0;
    list<PncRoute>::iterator itList;
    for( itList = routes->begin(); itList != routes->end(); ){
        if((*itList).on_route){
            laneIndex = i;
            break;
        }
        i++;
        itList++;
    }

        // swapping condition
    if (laneIndex != 0){
        // 2. swap routes[0] and routes[laneIndex]  -- this works for any size of 'routes'
        PncRoute temp1 = routes->front();
        PncRoute temp2;
        routes->pop_front();
        list<PncRoute>::iterator itList;
        int i = 0;
        for( itList = routes->begin(); itList != routes->end(); )
        {
            if(i == laneIndex - 1)
            {
               temp2 = *itList;  // store routes[laneIndex] in temp2
               routes->erase(itList);
               break;
            }
            else
               itList++;
            i++;
        }

        routes->insert(begin(*routes), temp2);

        int tag = 0;
        i = 0;
        for( itList = routes->begin(); itList != routes->end(); )
        {
            if(i == laneIndex)
            {
                tag = 1;
                routes->insert(itList, temp1);
                break;
            }
            else
               itList++;
            i++;
        }
        if(!tag)
            routes->insert(routes->end(), temp1);
    }

    cout << "Outside swap routes. " << endl;

}


void PncMap::substituteRoute(PncRoute new_routeSeg, list<PncRoute> *const routes, int route_id){

    if (routes->size() < 1)
        cout << "Invalid Argument in Function substituteRoute of Class PncMap: size of argument 'routes' is not greator than 0. " << endl;

    // erase element at index route_id
    list<PncRoute>::iterator itList;
    for( itList = routes->begin(); itList != routes->end(); )
    {
        if((*itList).route.id == route_id)
        {
           routes->erase(itList);
           break;
        }
        else
           itList++;
    }
    // insert routeSeg at index route_id
    int tag = 0;
    for( itList = routes->begin(); itList != routes->end(); )
    {
        if((*itList).route.id == route_id)
        {
            tag = 1;
            routes->insert(itList, new_routeSeg);
            break;
        }
        else
           itList++;
    }
    if(!tag)
        routes->insert(routes->end(), new_routeSeg);
}


void PncMap::getNearestPoint(Point3D point, const vector<MapPoint> points, int &nearest_index){
    double distance, min_distance = config_param_.MAX_NUMBER;
    for (int i = 0; i < points.size(); i++){
        distance = calDistance(point, points[i]);
        if (distance < min_distance){
            min_distance = distance;
            nearest_index = i;
        }
    }
}


bool PncMap::getShiftOnRoute(const Point3D &vehicle_state, const Route &route, double &vs_shift, int &behind_index){

cout<<"inside getShiftOnRoute function"<<endl;
    /// Add strict judgement of the arguments, and Error Handling Mechanism


    // Get the projected shift of vehicle on 'route'
    ///     -- WE DON'T TAKE CARE OF THE BORDER CASE FOR NOW
    ///        BECAUSE FOR THE CASES WE ARE DEALING WITH NOW, IN REALITY, THE BORDER CASE WILL NOT HAPPEN

    // 1. get point3D type of position from vehicle_state.
    Point3D point;
    point.x = vehicle_state.x;
    point.y = vehicle_state.y;

    // 2. get nearest point on the route
    int nearest_index;
    getNearestPoint(point, route.reference_points, nearest_index);
    behind_index = nearest_index - 1;
//    int forward_index=nearest_index+1;

    // maybe check if it exceeds the largest index too ?

    if (behind_index < 0){   /// CHACK HERE, BE AWARE OF THIS. THIS COULD CAUSE TROUBLE
        vs_shift = 0;
        behind_index = 0;
        return true;
    }
    if(nearest_index==route.reference_points.size()-1){

//        nearest_index=route.reference_points.size()-1;
         vs_shift=route.reference_points[nearest_index].s;
        return true;
    }

//    cout << "num of points: " << route.reference_points.size() << endl;

//    cout << "nearest_index + 1: " << nearest_index + 1 << endl;


//    cout << "Before calculating vs_shift. " << endl;

    // 3. use the second and the third nearest points to calculate the shift of the interpolated point
    // shift on route by doing vector dot product
    vs_shift = route.reference_points[behind_index].s +
            ((vehicle_state.x - route.reference_points[behind_index].point_enu.x) * (route.reference_points[nearest_index + 1].point_enu.x - route.reference_points[behind_index].point_enu.x)
            + (vehicle_state.y - route.reference_points[behind_index].point_enu.y) * (route.reference_points[nearest_index + 1].point_enu.y - route.reference_points[behind_index].point_enu.y))
            / (route.reference_points[nearest_index + 1].s - route.reference_points[behind_index].s);

    return true;
}


// Get the lane_id and the shift (on corresponding lane) -- we do this becasue there could be difference caused
// by using map points and route points to interpolate
void PncMap::getShiftOnLane(const vector<int> &laneIDs, int route_id, double vs_shift, double p1_s, int behind_index,
                            int &lane_id, double &p_shift){
cout<<"inside getShiftOnLane function"<<endl;
    // get the corresponding ids and then get the behind index
    int j = 0, id1=0, id2=0;
    for(int k = 0; k < (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1; k++){
        if (k != 0 && (*shifts_)[route_id].at(k) < (*shifts_)[route_id].at(k-1))
            j++;
        if (k == behind_index)
            id1 = laneIDs[j];
        if (k == behind_index + 2 || k==(config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE)
            id2 = laneIDs[j];
    }

    // get lane id where the vehicle is on (using the index to search in laneIDs[i])
    if (id1 == id2 || ((*shifts_)[route_id].at(behind_index) + vs_shift - p1_s)
            <= (base_map_->lanes[id1 - 1].central_points[base_map_->lanes[id1 - 1].central_points_num - 1].s + 0.0001))
        lane_id = id1;
    else
        lane_id = id2;

    // get the shift on the lane
    if(lane_id == id1)
        p_shift = (*shifts_)[route_id].at(behind_index) + vs_shift - p1_s;
    else
        p_shift = (*shifts_)[route_id].at(behind_index) + vs_shift - p1_s
                -  base_map_->lanes[id1 - 1].central_points[base_map_->lanes[id1 - 1].central_points_num - 1].s;

}


void PncMap::getStation(int id, int index, double s, const vector<double> &shifts, Route &route){
    Station route_station;
    Point3D point;
    int nearest_index=0;
    int last_ID = route.lanes_id.back();
    vector<MapPoint> points = route.reference_points;

    route.station.id = -1;
    route_station = base_map_->lanes[id - 1].station;
    if (route_station.id != -1 && s < route_station.end_s){
        if(id != last_ID || route_station.start_s < shifts.back()){
            // calculate the shift of start_s on route
            // 1.1 get the projected point of "start_s"
            point = getPoint3DFromShift(route_station.start_s, id - 1);
            // 1.2 linear search (use coordinates) for the nearest point in route
            getNearestPoint(point, points, nearest_index);
            if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
            if (nearest_index == 0)
               route_station.start_s = 0;
            else
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_station.start_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);

            if (id != last_ID || route_station.end_s < shifts.back()){
                // calculate the shift of end_s on route
                // 1.1 get the projected point of "start_s"
                point = getPoint3DFromShift(route_station.end_s, id - 1);
                // 1.2 linear search (use coordinates) for the nearest point in route
                getNearestPoint(point, points, nearest_index);
                if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                    nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                if (nearest_index == 0)
                    nearest_index = 1;
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_station.end_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);
            }
            else
                // assign the length of the route to end_s
                route_station.end_s = config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE;

            // assign the station to route.station
            route.station = route_station;
        }
    }

    int i;
    while (index != route.lanes_id.size() - 1){  // traverse route.lanes_id
        i = route.lanes_id[++index] - 1;
        if (route.station.id != -1)
            break;
        if (base_map_->lanes[i].station.id != -1){
            route_station = base_map_->lanes[i].station;
            if(i + 1 != last_ID || route_station.start_s < shifts.back()){
                // change the start/end shift for 'route_station'
                // 1.1 get the projected point of "start_s"
                point = getPoint3DFromShift(route_station.start_s, i);
                // 1.2 linear search (use coordinates) for the nearest point in route
                getNearestPoint(point, points, nearest_index);
                if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                    nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_station.start_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);

                if (i + 1 != last_ID || route_station.end_s < shifts.back()){
                    // calculate the shift of end_s on route
                    // 1.1 get the projected point of "start_s"
                    point = getPoint3DFromShift(route_station.end_s, i);
                    // 1.2 linear search (use coordinates) for the nearest point in route
                    getNearestPoint(point, points, nearest_index);
                    if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                      nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                    // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                    route_station.end_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);
                }
                else
                  // assign the length of the route to end_s
                  route_station.end_s = config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE;

                // assign route_station to route.station
                route.station = route_station;
            }
        }
    }

    ////////////////////////// Print out station info ///////////////////////////////
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    if(route.station.id == -1)
//        cout << "There's no station area on the route. " << endl;
//    else{
//        cout << "route.station.id: " << route.station.id << endl;
//        cout <<  "route.station.start_s: " << route.station.start_s << endl;
//        cout <<  "route.station.end_s: " << route.station.end_s << endl;
//    }
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;

}

void PncMap::getTerminal(int id, int index, double s, const vector<double> &shifts, Route &route){
    Terminal route_terminal;
    Point3D point;
    int nearest_index=0;
    int last_ID = route.lanes_id.back();
    vector<MapPoint> points = route.reference_points;

    route.terminal.id = -1;
    route_terminal = base_map_->lanes[id - 1].terminal;
    if (route_terminal.id != -1 && s < route_terminal.end_s){
        if(id != last_ID || route_terminal.start_s < shifts.back()){
            // calculate the shift of start_s on route
            // 1.1 get the projected point of "start_s"
            point = getPoint3DFromShift(route_terminal.start_s, id - 1);
            // 1.2 linear search (use coordinates) for the nearest point in route
            getNearestPoint(point, points, nearest_index);
            if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
            // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
            route_terminal.start_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);

            // calculate the shift of end_s on route
            if (id != last_ID || route_terminal.end_s < shifts.back()){
                // 1.1 get the projected point of "end_s"
                point = getPoint3DFromShift(route_terminal.end_s, id - 1);
                // 1.2 linear search (use coordinates) for the nearest point in route
                getNearestPoint(point, points, nearest_index);
                if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                    nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_terminal.end_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);
            }
            else
                // assign the length of the route to end_s
                route_terminal.end_s = config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE;

            // assign the terminal to route.terminal
            route.terminal = route_terminal;
        }
    }

    int i;
    while (index != route.lanes_id.size() - 1){  // traverse route.lanes_id
        i = route.lanes_id[++index] - 1;
        if (route.terminal.id != -1)
            break;
        if (base_map_->lanes[i].terminal.id != -1){
            route_terminal = base_map_->lanes[i].terminal;
            if(i + 1 != last_ID || route_terminal.start_s < shifts.back()){
                // overwrite the start/end shift for 'route_terminal'
                // 1.1 get the projected point of "start_s"
                point = getPoint3DFromShift(route_terminal.start_s, i);
                // 1.2 linear search (use route_terminal) for the nearest point in route
                getNearestPoint(point, points, nearest_index);
                if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                    nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_terminal.start_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);

                if (i + 1 != last_ID || route_terminal.end_s < shifts.back()){
                    // 1.1 get the projected point of "end_s"
                    point = getPoint3DFromShift(route_terminal.end_s, i);
                    // 1.2 linear search (use coordinates) for the nearest point in route
                    getNearestPoint(point, points, nearest_index);
                    if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                      nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                    // 1.3 interpolate using the second and the third nearest points to get the "end_s" on the route
                    route_terminal.end_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);
                }
                else
                  // assign the length of the route to end_s
                  route_terminal.end_s = config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE;

                // assign route_station to route.station
                route.terminal = route_terminal;
            }
        }
    }

    ////////////////////////// Print out terminal info ///////////////////////////////
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    if(route.terminal.id == -1)
//        cout << "There's no terminal area on the route. " << endl;
//    else{
//        cout << "route.terminal.id: " << route.terminal.id << endl;
//        cout <<  "route.terminal.start_s: " << route.terminal.start_s << endl;
//        cout <<  "route.terminal.end_s: " << route.terminal.end_s << endl;
//    }
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;
//    cout << "-------------------------------------------" << endl;

}


void PncMap::getTrafficLight(int id, int index, double s, const vector<double> &shifts, Route &route){
    TrafficLight route_traffic_light;
    Point3D point;
    int nearest_index=0;
    int last_ID = route.lanes_id.back();
    vector<MapPoint> points = route.reference_points;

    route.traffic_light.has_traffic_light = false;

    if (base_map_->lanes[id - 1].traffic_light_info.num_light_groups
            && s < base_map_->lanes[id - 1].traffic_light_info.end_s){
        if(id != last_ID || base_map_->lanes[id - 1].traffic_light_info.start_s < shifts.back()){
            route_traffic_light.has_traffic_light =  true;

            // calculate the shift of start_s on route
            // 1.1 get the corresponding point of "start_s"
            point = getPoint3DFromShift(base_map_->lanes[id - 1].traffic_light_info.start_s, id - 1);
            // 1.2 linear search (using coordinates) for the nearest point in route
            getNearestPoint(point, points, nearest_index);
            if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
            // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
            route_traffic_light.start_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);

            // calculate the shift of end_s on route
            if (id != last_ID || base_map_->lanes[id - 1].traffic_light_info.end_s < shifts.back()){
                // 1.1 get the corresponding point of "end_s"
                point = getPoint3DFromShift(base_map_->lanes[id - 1].traffic_light_info.end_s, id - 1);
                // 1.2 linear search (using coordinates) for the nearest point in route
                getNearestPoint(point, points, nearest_index);
                if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                    nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_traffic_light.end_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);
            }
            else
                // assign the length of the route to end_s
                route_traffic_light.end_s = config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE;

            // assign ids of traffic_light_units  -- more logics to be added?
            for (TrafficLightGroup group: base_map_->lanes[id - 1].traffic_light_info.light_groups){
                for (TrafficLightUnit unit: group.light_units){
                    if ( (unit.light_type == 1 || unit.light_type == 4 || unit.light_type == 5 || unit.light_type == 8)
                         && route.turn_type == 1 ||   // STRAIGHT
                            (unit.light_type == 2 || unit.light_type == 4 || unit.light_type == 6 || unit.light_type == 8)
                         && route.turn_type == 2 ||
                            (unit.light_type == 3 || unit.light_type == 7) && route.turn_type == 3)
                        route_traffic_light.ids.push_back(unit.id);
                }
            }

            // assign to the attribute of route
            route.traffic_light = route_traffic_light;
        }
    }

    int i;
    while (index != route.lanes_id.size() - 1){  // traverse route.lanes_id
        i = route.lanes_id[++index] - 1;
        if (route.traffic_light.has_traffic_light)
            break;
        if (base_map_->lanes[i].traffic_light_info.num_light_groups){
            route_traffic_light.has_traffic_light =  true;
            if(i + 1 != last_ID || base_map_->lanes[i].traffic_light_info.start_s < shifts.back()){
                // change the start/end shift for 'route_traffic_light_info'
                // 1.1 get the corresponding point of "start_s"
                point = getPoint3DFromShift(base_map_->lanes[i].traffic_light_info.start_s, i);
                // 1.2 linear search (using coordinates) for the nearest point in route
                getNearestPoint(point, points, nearest_index);
                if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                    nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                route_traffic_light.start_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);

                if (i+1 != last_ID || base_map_->lanes[i].traffic_light_info.end_s < shifts.back()){
                    // calculate the shift of end_s on route
                    // 1.1 get the corresponding point of "end_s"
                    point = getPoint3DFromShift(base_map_->lanes[i].traffic_light_info.end_s, i);
                    // 1.2 linear search (using coordinates) for the nearest point in route
                    getNearestPoint(point, points, nearest_index);
                    if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                        nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
                    // 1.3 interpolate using the second and the third nearest points to get the "start_s" on the route
                    route_traffic_light.end_s = getShiftByInterpolation(points[nearest_index - 1], points[nearest_index + 1], point);
                }
                else
                    // assign the length of the route to end_s
                    route_traffic_light.end_s = config_param_.LOOK_FORWARD_DISTANCE + config_param_.LOOK_BACKWARD_DISTANCE;

                // assign ids of traffic_light_units  -- more logics to be added?
                for (TrafficLightGroup group: base_map_->lanes[i].traffic_light_info.light_groups){
                    for (TrafficLightUnit unit: group.light_units){
                        if ( (unit.light_type == 1 || unit.light_type == 4 || unit.light_type == 5 || unit.light_type == 8)
                             && route.turn_type == 1 ||   // STRAIGHT
                                (unit.light_type == 2 || unit.light_type == 4 || unit.light_type == 6 || unit.light_type == 8)
                             && route.turn_type == 2 ||
                                (unit.light_type == 3 || unit.light_type == 7) && route.turn_type == 3)
                            route_traffic_light.ids.push_back(unit.id);
                    }
                }

                // assign to the attribute of route
                route.traffic_light = route_traffic_light;

            }
        }
    }

    ////////////////////////// Print out traffic light info ///////////////////////////////
//    cout << "-------------------------------------------" << endl;
//    if (route.overlap_zone.size() == 0)
//        cout << "There's no traffic light area. " << endl;
//    else{
//        cout << "size of overlap_zone: " << route.overlap_zone.size() << endl;
//        cout << "route.overlap_zone.front().id: " << route.overlap_zone.front().id << endl;
//        cout <<  "route_traffic_light_info.start_s: " << route_traffic_light_info.start_s << endl;
//        cout <<  "route_traffic_light_info.end_s: " << route_traffic_light_info.end_s << endl;
//        cout << "-------------------------------------------" << endl;
//    }

}


//void PncMap::getWidth(const vector<double> &shifts, Route &route){

//    int initial_lane_index = route.lanes_id[0] - 1;
//    vector<MapPoint> points = route.reference_points;
//    Width width_point;
//    double distance;
//    double minDis_left = config_param_.MAX_NUMBER, minDis_right = config_param_.MAX_NUMBER;
//    int pointIndex_left = -1, pointIndex_right = -1;

//    route.widths.clear();

//    // [for the first point]
//    // find the nearest point index:
//    // for left:
//    for (int j = 0; j < base_map_->lanes[initial_lane_index].left_boundary.point_num; j++){
//      distance = calDistance(base_map_->lanes[initial_lane_index].left_boundary.points[j], points[0]);
//      if (distance < minDis_left){
//          minDis_left = distance;
//          pointIndex_left = j;
//      }
//    }
//    // for right:
//    for (int j = 0; j < base_map_->lanes[initial_lane_index].right_boundary.point_num; j++){
//      distance = calDistance(base_map_->lanes[initial_lane_index].right_boundary.points[j], points[0]);
//      if (distance < minDis_right){
//          minDis_right = distance;
//          pointIndex_right = j;
//      }
//    }
//    // store in width_point
//    width_point.s = 0.;
//    width_point.center_to_left = minDis_left;
//    width_point.center_to_right = minDis_right;
//    // store in vector widths
//    route.widths.push_back(width_point);


//    // [for the other points (from the second)]
//    //   -- compare with points in front of p, until you find a larger one, choose the former one
//    int left_lane_index = 0, right_lane_index = 0, frontLaneIndex, frontPointIndex;
//    int current_left_lane_index, current_pointIndex_left, current_right_lane_index, current_pointIndex_right;
//    for (int i = 1; i < shifts.size(); i++){
//        // store shift in width
//        width_point.s = points[i].s;

//        // FOR LEFT:
//        // 1. find the points (get nearest and then get neighbors)
//        minDis_left = calDistance(base_map_->lanes[route.lanes_id[left_lane_index] - 1].left_boundary.points[pointIndex_left], points[i]);
//        current_left_lane_index = left_lane_index;
//        current_pointIndex_left = pointIndex_left;
//        for (int k = 0; k < 5; k++){
//            getLeftFrontPointIndex(current_pointIndex_left, current_left_lane_index, route.lanes_id, frontLaneIndex, frontPointIndex);
//            distance = calDistance(base_map_->lanes[route.lanes_id[frontLaneIndex] - 1].left_boundary.points[frontPointIndex], points[i]);
//            current_pointIndex_left = frontPointIndex;
//            current_left_lane_index = frontLaneIndex;
//            if (distance < minDis_left){
//                pointIndex_left = frontPointIndex;
//                left_lane_index = frontLaneIndex;
//                minDis_left = distance;
//            }
//        }

//        // 2. use vector cross product to get distance directly and store in width_point
//        width_point.center_to_left = minDis_left;

//        // FOR RIGHT:
//        // 1. find the points (get nearest and then get neighbors)
//        minDis_right = calDistance(base_map_->lanes[route.lanes_id[right_lane_index] - 1].right_boundary.points[pointIndex_right], points[i]);
//        current_right_lane_index = right_lane_index;
//        current_pointIndex_right = pointIndex_right;
//        for (int k = 0; k < 5; k++){
//            getRightFrontPointIndex(current_pointIndex_right, current_right_lane_index, route.lanes_id, frontLaneIndex, frontPointIndex);
//            distance = calDistance(base_map_->lanes[route.lanes_id[frontLaneIndex] - 1].right_boundary.points[frontPointIndex], points[i]);
//            current_pointIndex_right = frontPointIndex;
//            current_right_lane_index = frontLaneIndex;
//            if (distance < minDis_right){
//                pointIndex_right = frontPointIndex;
//                right_lane_index = frontLaneIndex;
//                minDis_right = distance;
//            }
//        }
//        // 2. use vector cross product to get distance directly and store in width_point
//        width_point.center_to_right = minDis_right;

//        // store width_point to widths
//        route.widths.push_back(width_point);
//    }

//    ////// show width
////    cout << "width: " << endl;
////    for (Width width: route.widths){
////        cout << "(" << width.center_to_left << ", " << width.center_to_right << ")" << endl;
////    }

//}


//void PncMap::getWidth( vector<double> &shifts, Route &route){

//    vector<MapPoint> points = route.reference_points;
//    Width width_point;
//    int lane_id_index = 0;
//    double total_left_min_width=std::numeric_limits<double>::max();
//    double total_right_min_width=std::numeric_limits<double>::max();

//    route.widths.clear();
////    route.road_widths.clear();
//    auto temp_status=PncInformation::Instance()->GetPncStatus().widths_status;

//    for (int i = 0; i < shifts.size(); i++){

//        if (i != 0 && shifts[i] < shifts[i-1]){
//            lane_id_index++;
//        }

//        // store shift in width
//        width_point.s = points[i].s;

//        // FOR LEFT:
//        auto  current_lane_id=route.lanes_id[lane_id_index];
////        int     current_point_index=shifts[i]/0.5;
//                auto  current_lane_id=route.lanes_id[lane_id_index];
//                auto current_lane=base_map_->lanes[current_lane_id-1];
//                auto current_lane_point_num=current_lane.central_points_num;
////                int current_point_index=0;

//        //lane left and center distance
//        cout<<"lane left boundary and center distance: "<<endl;
//        for(int test_index=0;test_index<current_lane_point_num;test_index++){
//            auto test_distance=calDistance(current_lane.left_boundary.points[current_point_index],
//                              current_lane.central_points[current_point_index]  );
//            cout<<"distance is "<<test_distance<<endl;
//        }
//        cout<<"lane left boundary and center distance test end "<<endl;




//      auto temp_left_width   =
//                calDistance(base_map_->lanes[route.lanes_id[lane_id_index] - 1].left_boundary.points[(int)(shifts[i] / config_param_.STEP_SIZE)],
//                   // points[i]
//                    base_map_->lanes[route.lanes_id[lane_id_index] - 1].central_points[(int)(shifts[i] / config_param_.STEP_SIZE)]

//                );
//      if(temp_left_width>6){
//          PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_LEFT_WIDTHS_OUT_OF_RANGE;
//          cout<<"Error left width is "<<temp_left_width<<endl;
//          temp_left_width=6;

//      }
//      width_point.center_to_left=temp_left_width;

//        // FOR RIGHT:
//      auto temp_right_width =
//                calDistance(base_map_->lanes[route.lanes_id[lane_id_index] - 1].right_boundary.points[(int)(shifts[i] / config_param_.STEP_SIZE)],
//                //    points[i]
//                 base_map_->lanes[route.lanes_id[lane_id_index] - 1].central_points[(int)(shifts[i] / config_param_.STEP_SIZE)]

//                );
//      if(temp_right_width>6){

//          PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_RIGHT_WIDTHS_OUT_OF_RANGE;
//          cout<<"Error right width is "<<temp_right_width<<endl;
//          temp_right_width=6;

//      }
//      width_point.center_to_right=temp_right_width;
//        // store width_point to widths
//        route.widths.push_back(width_point);
////        route.road_widths.push_back(width_point);
//    }

//    ////// show width
//    cout << "width: " << endl;
//    for (auto width: route.widths){
//        auto current_point_left_min_width=width.center_to_left;
//        auto current_point_right_min_width=width.center_to_right;
//        if(current_point_left_min_width<total_left_min_width){
//            total_left_min_width=current_point_left_min_width;
//        }
//        if(current_point_right_min_width<total_right_min_width){
//            total_right_min_width=current_point_right_min_width;
//        }
//        cout << "(" << width.center_to_left << ", " << width.center_to_right << ")" << endl;
//    }
//    if(total_left_min_width<0.2){
//        PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_LEFT_WIDTHS_MATCH_ERROR;
//        cout<<"lane left  error width is "<<total_left_min_width<<endl;
//    }
//    if(total_right_min_width<0.2){
//        PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_RIGHT_WIDTHS_MATCH_ERROR;
//        cout<<"lane right  error width is "<<total_right_min_width<<endl;
//    }

//}

//new getwidth 0317
void PncMap::getWidth( vector<double> &shifts, Route &route){

    vector<MapPoint> points = route.reference_points;
    Width width_point;
    int lane_id_index = 0;
    double total_left_min_width=std::numeric_limits<double>::max();
    double total_right_min_width=std::numeric_limits<double>::max();

    route.widths.clear();
//    route.road_widths.clear();
    auto temp_status=PncInformation::Instance()->GetPncStatus().widths_status;

    for (int i = 0; i < shifts.size(); i++){

        if (i != 0 && shifts[i] < shifts[i-1]){
            lane_id_index++;
        }

        // store shift in width
        width_point.s = points[i].s;



        auto  current_lane_id=route.lanes_id[lane_id_index];
        auto current_lane=base_map_->lanes[current_lane_id-1];
        int current_point_index=PncUtil::GetMatchedIndexOnLane(current_lane,shifts[i]);

//        //lane left and center distance
//        cout<<"lane left boundary and center distance: "<<endl;
//        for(int test_index=0;test_index<current_lane_point_num;test_index++){
//            auto test_left_distance=calDistance(current_lane.left_boundary.points[test_index],
//                              current_lane.central_points[test_index]  );
//            auto test_right_distance=calDistance(current_lane.right_boundary.points[test_index],
//                              current_lane.central_points[test_index]  );
//            cout<<"left distance is "<<test_left_distance<<",  right distance is  "<<test_right_distance<<endl;
//        }
//        cout<<"lane left boundary and center distance test end "<<endl;


        // FOR LEFT:
      auto temp_left_width   =
                calDistance(current_lane.left_boundary.points[current_point_index],
                                  current_lane.central_points[current_point_index]  );

      if(temp_left_width>6){
          PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_LEFT_WIDTHS_OUT_OF_RANGE;
          cout<<"Error left width is "<<temp_left_width<<endl;
          temp_left_width=6;

      }
      width_point.center_to_left=temp_left_width;

        // FOR RIGHT:
      auto temp_right_width =
                calDistance( current_lane.right_boundary.points[current_point_index],
                             current_lane.central_points[current_point_index]   );
      if(temp_right_width>6){

          PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_RIGHT_WIDTHS_OUT_OF_RANGE;
          cout<<"Error right width is "<<temp_right_width<<endl;
          temp_right_width=6;

      }
      width_point.center_to_right=temp_right_width;
        // store width_point to widths
        route.widths.push_back(width_point);
//        route.road_widths.push_back(width_point);
    }

    ////// show width
    cout << "width: " << endl;
    for (auto width: route.widths){
        auto current_point_left_min_width=width.center_to_left;
        auto current_point_right_min_width=width.center_to_right;
        if(current_point_left_min_width<total_left_min_width){
            total_left_min_width=current_point_left_min_width;
        }
        if(current_point_right_min_width<total_right_min_width){
            total_right_min_width=current_point_right_min_width;
        }
        cout << "(" << width.center_to_left << ", " << width.center_to_right << ")" << endl;
    }
    if(total_left_min_width<0.2){
        PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_LEFT_WIDTHS_MATCH_ERROR;
        cout<<"lane left  error width is "<<total_left_min_width<<endl;
    }
    if(total_right_min_width<0.2){
        PncInformation::Instance()->MutableGetPncStatus()->widths_status=PncInformation::WidthStatus::LANE_RIGHT_WIDTHS_MATCH_ERROR;
        cout<<"lane right  error width is "<<total_right_min_width<<endl;
    }

}

//0219 added
//void PncMap::SetRoadWidth(  list<PncRoute>& routes){

//    double lane_width;
//    vector<double> lane_widths;
//    vector<pair<int,vector<double>>>  id_widths_pairs;        //<route_id ,  lane_widths>

//    for(auto pnc_route:routes){

//        lane_widths.clear();
//            for(int i=0;i<pnc_route.route.widths.size();i++){
//               auto temp_left_width=pnc_route.route.widths[i].center_to_left;
//               auto temp_right_width=pnc_route.route.widths[i].center_to_right;
//               lane_width=temp_left_width+temp_right_width;
//               lane_widths.push_back(lane_width);
//            }
//            id_widths_pairs.push_back(std::make_pair(pnc_route.route.id, lane_widths));
//        }

//    list<PncRoute>::iterator iter;
//    Width road_width_point;
//    list<PncRoute>::reverse_iterator rev_iter;

//    for(iter=routes.begin();iter!=routes.end();iter++){


//        if(iter==routes.begin()){
//            auto&  temp_route=*iter;
//            auto     temp_points_num=temp_route.route.widths.size();

//            for(int i=0;i<temp_points_num;i++){
//                 road_width_point.s=temp_route.route.widths[i].s;
//                 road_width_point.center_to_left=temp_route.route.widths[i].center_to_left;
//                 road_width_point.center_to_right=temp_route.route.widths[i].center_to_right+id_widths_pairs[1].second.at(i);
//                 temp_route.route.road_widths.push_back(road_width_point);
//            }
//        }
//        else {
//            auto& temp_route=*iter;
//            auto   temp_points_num=temp_route.route.widths.size();

//            for(int i=0;i<temp_points_num;i++){
//                  road_width_point.s=temp_route.route.widths[i].s;
//                  road_width_point.center_to_left=temp_route.route.widths[i].center_to_left+id_widths_pairs[0].second.at(i);
//                  road_width_point.center_to_right=temp_route.route.widths[i].center_to_right;
//                  temp_route.route.road_widths.push_back(road_width_point);
//            }
//        }

////        //left search
////        while((--temp_iter)!=routes.begin()){

////            temp_iter--;
////        }

////        //right search
////        while(temp_iter!=routes.end()){
////            temp_iter++;
////        }
//}

//    for(auto& current_pnc_route:routes){
//        if(current_pnc_route.route.id==0){

//                        auto     temp_points_num=current_pnc_route.route.widths.size();

//                        for(int i=0;i<temp_points_num;i++){
//                             road_width_point.s=current_pnc_route.route.widths[i].s;
//                             road_width_point.center_to_left=current_pnc_route.route.widths[i].center_to_left;
//                             road_width_point.center_to_right=current_pnc_route.route.widths[i].center_to_right+id_widths_pairs[1].second.at(i);
//                             current_pnc_route.route.road_widths.push_back(road_width_point);
//                        }
//        }

//       else if(current_pnc_route.route.id==1){

//                        auto     temp_points_num=current_pnc_route.route.widths.size();

//                        for(int i=0;i<temp_points_num;i++){
//                             road_width_point.s=current_pnc_route.route.widths[i].s;
//                             road_width_point.center_to_left=current_pnc_route.route.widths[i].center_to_left+id_widths_pairs[0].second.at(i);
//                             road_width_point.center_to_right=current_pnc_route.route.widths[i].center_to_right;
//                             current_pnc_route.route.road_widths.push_back(road_width_point);
//                        }
//        }
//    }

//return;

//}

void PncMap::getDestination(const vector<int> &lane_IDs, const vector<int> &destination_lanes, Point3D destination, Route &route){
    Destination route_destination;
    vector<int> intersection;
    double min_dis = config_param_.MAX_NUMBER, distance;
    int nearest_index=0, index = 0;

    route_destination.has_destination = false;
    calIntersection(lane_IDs, destination_lanes, intersection);

    if (intersection.size() > 0){
        // check if the destination is on the route (if the min dis between a reference point and the destination is smaller than width, then it is on route)
        for (MapPoint point: route.reference_points) {   // YOU CAN CHANGE DIS TO BE THE DISTANCE OF "DESTINATION TO THE LINE" DEFINED BY
            distance = calDistance(destination, point);  //     NEIGHBOR POINTS OF THE NREAREST POINT ON ROUTE TO THE DESTINATION
            if (distance < min_dis){
                min_dis = distance;
                nearest_index = index;
            }
            index++;
        }

        // if on route, then assign: has_destination to be true; start_s to be the projected shift of destination on route; end_s, position.
        if (min_dis < sqrt(2) * route.widths[0].center_to_left) {
            route_destination.has_destination = true;
            route_destination.position = destination;

            // start_s
            if (nearest_index == (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE + 1)
                nearest_index = (config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE) / config_param_.STEP_SIZE;
            else if (nearest_index == 0)
                nearest_index = 1;
            // interpolate using the second and third nearest points to get the "start_s" on the route
            route_destination.start_s = getShiftByInterpolation(route.reference_points[nearest_index - 1], route.reference_points[nearest_index + 1], destination);

            // end_s
            if (route_destination.start_s + 5 < config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE)
                route_destination.end_s = route_destination.start_s + 5;
            else
                route_destination.end_s = config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE;
        }
    }

    route.destination = route_destination;
}


Route PncMap::generateRoute(int id, vector<MapPoint> &points, const vector<int> &lane_IDs, double s,
                             vector<double> &shifts, Point3D destination, const vector<int> &destination_lanes){

    // check legality of arguments
    if (points.size() <= 0)
        cout << "Invalid Argument in Function generateRoute of Class PncMap: size of argument 'points' is not greator than 0. " << endl;
    if (lane_IDs.size() <= 0)
        cout << "Invalid Argument in Function generateRoute of Class PncMap: size of argument 'lane_IDs' is not greator than 0. " << endl;
    if (shifts.size() <= 0)
        cout << "Invalid Argument in Function generateRoute of Class PncMap: size of argument 'shifts' is not greator than 0. " << endl;

    // declare variable
    Route route;
    int index=0;   // of id in lane_IDs

    // overwrite shift values of the points
    for (double i = 0.; i < points.size(); i++)
      points[i].s = i * config_param_.STEP_SIZE;

    // assign points and lane ids
    route.reference_points = points;
    route.lanes_id = lane_IDs;

    /// turn_type (choose the one in current lane)
    // [you haven't assign value to this attribute]
    route.turn_type = base_map_->lanes[id - 1].turn_type;

    // find the index of id in lane_IDs
    for (int i = 0; i < lane_IDs.size(); i++){
        if (id == lane_IDs[i])
            index = i;
    }

    /// station (choose the nearest valid station of the lane where the vehicle is in if the vehicle position is in front of that;
    // [ we only need to assign ONE station here ]
    getStation(id, index, s, shifts, route);

    // terminal
    getTerminal(id, index, s, shifts, route);

    /// traffic light
    getTrafficLight(id, index, s, shifts, route);

    /// speed_limit (choose the nearest in the front)
    route.speed_limit = base_map_->lanes[id - 1].speed_limit;
   
    /// width (for each point on route: find nearest two points on boundary and calculate the distance)
    getWidth(shifts, route);

    /// desitnation
    getDestination(lane_IDs, destination_lanes, destination, route);

//    auto temp_in=PncInformation::Instance()->GetPncStatus().widths_status;

    return route;
}


// lane_id, points, lane_IDs, p_shift, shifts
Route PncMap::generateRouteTmp(int id, vector<MapPoint> &points, const vector<int> &lane_IDs,
                                   double s,  vector<double> &shifts){

    // check legality of arguments
    if (points.size() <= 0)
        cout << "Invalid Argument in Function generateRoute of Class PncMap: size of argument 'points' is not greator than 0. " << endl;
    if (lane_IDs.size() <= 0)
        cout << "Invalid Argument in Function generateRoute of Class PncMap: size of argument 'lane_IDs' is not greator than 0. " << endl;
    if (shifts.size() <= 0)
        cout << "Invalid Argument in Function generateRoute of Class PncMap: size of argument 'shifts' is not greator than 0. " << endl;

    // declare variable
    Route route;
    int index=0;   // of id in lane_IDs

    // overwrite shift values of the points
    for (double i = 0.; i < points.size(); i++)
      points[i].s = i * config_param_.STEP_SIZE;

    // assign points and lane ids
    route.reference_points = points;
    route.lanes_id = lane_IDs;

    /// turn_type (choose the one in current lane)
    // [you haven't assign value to this attribute]
    route.turn_type = base_map_->lanes[id - 1].turn_type;

    // find the index of id in lane_IDs
    for (int i = 0; i < lane_IDs.size(); i++){
        if (id == lane_IDs[i])
            index = i;
    }

    /// station (choose the nearest valid station of the lane where the vehicle is in if the vehicle position is in front of that;
    // [ we only need to assign ONE station here ]
    getStation(id, index, s, shifts, route);

    // terminal
    getTerminal(id, index, s, shifts, route);   // similar to getStation.

    /// traffic light
    getTrafficLight(id, index, s, shifts, route);

    /// speed_limit (choose the nearest in the front)
//    if (s + config_param_.MAX_DISTANCE < base_map_->lanes[id - 1].central_points[base_map_->lanes[id - 1].central_points_num - 1].s)
      route.speed_limit = base_map_->lanes[id - 1].speed_limit;
//    else
//      route.speed_limit = base_map_->lanes[lane_IDs[index + 1] - 1].speed_limit;

    /// width (for each point on route: find nearest two points on boundary and calculate the distance)
    getWidth(shifts, route);
//    auto temp_in=PncInformation::Instance()->GetPncStatus().widths_status;
    return route;
}



void PncMap::checkLeftLanes(int id, vector<int> &lane_ids, vector<double> &projectedShifts, Point3D vehicle_state){

//    cout << "Inside checkLeftLanes. " << endl;

    int lane_id = id;     // for while loops going leftwards
    pair<int, int> laneID_pointIndex;
    double distance, minDis = config_param_.MAX_NUMBER;
    MapPoint p1;
    MapPoint p2;
    pair<int, double> id_shift;

    // check the lanes leftwards one by one (stop when a lane which can't be entered is found)
    while (base_map_->lanes[lane_id - 1].left_boundary.type == 2){

//        cout << "Inside while loop. " << endl;

//        cout << "id: " << id << endl;
//        cout << ", and its left boundary type: " << base_map_->lanes[lane_id - 1].left_boundary.type << endl;

        // change lane id
        lane_id = base_map_->lanes[lane_id - 1].left_lane_id;

//        cout << "left_lane_id : " << lane_id << endl;


        laneID_pointIndex.first = lane_id;

        /// Get the nearest point on this lane --
        // calculate and compare all distances between the vehicle position and the points in the lane
        for (int j = 0; j < base_map_->lanes[lane_id - 1].central_points_num; j++){
            distance = calculateDistance(vehicle_state, base_map_->lanes[lane_id - 1].central_points[j]);
            if (distance < minDis){
                minDis = distance;
                laneID_pointIndex.second = j;
            }
        }

        /// THERE MIGHT BE A BUG HERE. -- WHAT IF lane_id and id_shift.first are different? Does it matter?

//        cout << "found min point index. " << endl;

        // get the projected shift for the vehicle
        getNeighborPoints(laneID_pointIndex.first - 1, laneID_pointIndex.second, p1, p2);
        id_shift = getProjectedShift(laneID_pointIndex.first - 1, laneID_pointIndex.second, p1, p2, vehicle_state);

//        cout << "got projected id and shift. " << endl;

        // append lane_id to lane_ids in the front
        lane_ids.insert(begin(lane_ids), id_shift.first);
        // append p to projectedPoints in the front
        projectedShifts.insert(begin(projectedShifts), id_shift.second);

//        cout << "fished one iteration. " << endl;

    } // end while loop LEFTWARDS

//    cout << "Getting outside checkLeftLanes. " << endl;

}



void PncMap::checkRightLanes(int id, vector<int> &lane_ids, vector<double> &projectedShifts, Point3D vehicle_state){

//    cout << "Inside checkLeftLanes. " << endl;

    int lane_id = id;     // for while loops going rightwards
    pair<int, int> laneID_pointIndex;
    double distance, minDis = config_param_.MAX_NUMBER;
    MapPoint p1;
    MapPoint p2;
    pair<int, double> id_shift;

    // check the lanes rightwards one by one (stop when a lane which can't be entered is found)
    while (base_map_->lanes[lane_id - 1].right_boundary.type == 2){

//        cout << "Inside while loop. " << endl;

        // change lane id
        lane_id = base_map_->lanes[lane_id - 1].right_lane_id;
        laneID_pointIndex.first = lane_id;

        /// Get the nearest point on this lane --
        // calculate and compare all distances between the vehicle position and the points in the lane
        for (int j = 0; j < base_map_->lanes[lane_id - 1].central_points_num; j++){
            distance = calculateDistance(vehicle_state, base_map_->lanes[lane_id - 1].central_points[j]);
            if (distance < minDis){
                minDis = distance;
                laneID_pointIndex.second = j;
            }
        }

        // get the projected point for the vehicle
        getNeighborPoints(laneID_pointIndex.first - 1, laneID_pointIndex.second, p1, p2);
        id_shift = getProjectedShift(laneID_pointIndex.first - 1, laneID_pointIndex.second, p1, p2, vehicle_state);

        // append lane_id to lane_ids in the end
        lane_ids.insert(end(lane_ids), id_shift.first);
        // append p to projectedPoints in the end
        projectedShifts.insert(end(projectedShifts), id_shift.second);

    } // end while loop RIGHTWARDS

//    cout << "Getting outside checkLeftLanes. " << endl;

}


// (for global planning in use)  -- this function may bring more than one route
bool PncMap::getShifts(double p_s, int id, const vector<vector<int>> &traversable_lanes,
                       vector<vector<double>> &list_shifts, vector<vector<int>> &list_lane_IDs, bool &near_destination){   // ADD NEAR_DESTINATION/TERMINAL...

    cout << "Inside getShifts. " << endl;

    double forward_s = p_s + config_param_.LOOK_FORWARD_DISTANCE;
    double backward_s = p_s - config_param_.LOOK_BACKWARD_DISTANCE;
    int currentID = id;
    double s;
    double flag = p_s;
    double length;
    int flag_end_back = 0, flag_end_forward = 0;
    vector<double> shifts_back, shifts;
    vector<int> lane_IDs_back, lane_IDs;
    vector<vector<int>> list_intersection;

    lane_IDs_back.push_back(currentID);
    /// for points BACKWARDS
    while (backward_s < 0.){
        // get the shifts of the points on this lane
        for(s = flag; s >= 0. - 0.001; s -= config_param_.STEP_SIZE)
            shifts_back.insert(shifts_back.begin(), s);
        // get the behind lane id and append in the front of the list
        if (base_map_->lanes[currentID - 1].num_behind_lanes <= 0){
            flag_end_back = 1;
            break;
        }
        currentID = base_map_->lanes[currentID - 1].behind_lane_ids[0];
        lane_IDs_back.insert(lane_IDs_back.begin(), currentID);
        // update length
        length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
        // add the length of the behind lane to backward_s
        backward_s += length;
        // change flag(end) to be inside the next lane
        flag = s + length;
    } // end while loop
    if(flag_end_back){   // for the case where the vehicle starts at some point where there're not enough distance to look backwards
        // reset loop variants
        forward_s = config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE;
        length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
        currentID = id;
    }
    else{
        // we are in the case where backward_s is greater than 0, now we need to obtain the last bit for the shifts
        for (s = flag; s >= backward_s - 0.0001; s -= config_param_.STEP_SIZE)
            shifts_back.insert(shifts_back.begin(), s);
        // reset loop variants
        currentID = id;
        length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
    }


    cout << "Just collected shifts backwards. " << endl;

    /// for points FORWARDS
    // 0. FIND the current_INDEX (IN FIRST DIMENSION) of "ids" in traversable_lanes
    int step_index=0;
     if (!findStepIndex(id, traversable_lanes, step_index)){
          cout << "Error: current id is not in traversable_lanes according to global path. " << endl;
          return false;
     }
     cout<<"cur step_index is "<<step_index<<endl;
     cout<<"current lane id is  "<<id<<endl;
     cout<<"---------------------------------------------------"<<endl;
     cout<<"current traversable lanes :"<<endl;
     for(auto traversable_lane:traversable_lanes){

        for(auto temp_lane:traversable_lane){
            cout<<temp_lane<<" ";
        }
                  cout<<endl;
     }
cout<<"---------------------------------------------------"<<endl;
    // 1. get the maximum size - "count" of INTERSECTION between front_lane_ids and the lane_ids in the each segment of traversable_lanes covered by length_of(forward+backward)
    //        and the INTERSECTION_LIST
    int count = 1, break_index = 0;
    if (!getIntersectionList(id, forward_s, length, traversable_lanes, step_index, count, list_intersection, break_index))
        near_destination = true;          // near destination -- normally this won't happend, but you still need to handle any case of error, just in case.
                              // Actually, you just initialize break_index with 0, then the program will automatically look forward in the front lane (even it's not in global path).

    if(list_intersection.size()>0){
        cout<<"current list_intersection size is "<<list_intersection.size()<<endl;
        cout<<"current list_intersection is "<<endl;
        for(int i=0;i<list_intersection.size();i++){
             for(int j=0;j<list_intersection[i].size();j++)
            cout<<list_intersection[i][j]<<" ";
        }
        cout<<endl;
    }
    else{
        cout<<"list_intersection size error"<<endl;
    }



    cout << "Just got TRAVERSABLE LANES.  " << endl;


    // 2. collect shifts forwards   - - write another function for this?
    int list_intersection_index;
    for (int index = 0; index < count; index++){
        // initializations
        if(flag_end_back){   // for the case where the vehicle starts at some point where there're not enough distance to look backwards
            // reset loop variants
            forward_s = config_param_.LOOK_BACKWARD_DISTANCE + config_param_.LOOK_FORWARD_DISTANCE;
            currentID = id;
            length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
            flag = 0.;
            // reinitialize collections
            lane_IDs.clear();
            lane_IDs.push_back(currentID);
            shifts.clear();
            shifts.push_back(flag);
        }
        else{
            // reset loop variants
            forward_s = p_s + config_param_.LOOK_FORWARD_DISTANCE;
            currentID = id;
            length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
            flag = p_s;
            // reinitialize collections
            lane_IDs = lane_IDs_back;
            shifts = shifts_back;
        }

        cout << "Collecting shifts forwards, just reinitiated.  " << endl;

        // collect data
        flag_end_forward = 0;
        list_intersection_index = 0;
        while (forward_s > length){
            // get the shifts of the points on this lane
            for (s = flag + config_param_.STEP_SIZE; s <= length; s += config_param_.STEP_SIZE)
                shifts.push_back(s);
            // change the value of forward_s
            forward_s -= length;
            // change flag(for start)
            flag = s - config_param_.STEP_SIZE - length;

            // change the forward lane id and append at the back of the list


            cout << "------------------------" << endl;
            cout << "list_intersection_index: " << list_intersection_index << endl;
            cout << "break_index: " <<  break_index << endl;

            
            if (list_intersection.size() >0)
                cout << "list_intersection[list_intersection_index].size is " << list_intersection[list_intersection_index].size() << endl;
            else {
                cout << "Error: Something wrong could be with the binary hdmap file or global path. " << endl;
                return false;
            }

            cout << "------------------------" << endl;


            if (list_intersection_index <= break_index && list_intersection[list_intersection_index].size() <= 0){ // what's the second case? I forgot.
                flag_end_forward = 1;

                cout << "------------------------" << endl;
                cout << "flag_end_forward: " << flag_end_forward << endl;
                cout << "------------------------" << endl;

                break;
            }
            else if (list_intersection_index > break_index)
                currentID = base_map_->lanes[currentID - 1].front_lane_ids[0];
            else if (list_intersection[list_intersection_index].size() > 1)
                currentID = list_intersection[list_intersection_index][index];
            else
                currentID = list_intersection[list_intersection_index][0];

            lane_IDs.push_back(currentID);
            // change loop variants
            length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
            list_intersection_index++;
        } // end while loop



        cout << "Collecting shifts forwards, outside while loop. " << endl;

        // collect the last bits for the list and append to list of shifts/lane_IDs
        if(!flag_end_forward){
            for (s = flag + config_param_.STEP_SIZE; s <= forward_s + 0.0001; s += config_param_.STEP_SIZE)
                shifts.push_back(s);
            list_shifts.push_back(shifts);
            list_lane_IDs.push_back(lane_IDs);
        }
    }

    cout << "getting outside getShifts. "  << endl;

    return true;
}


bool PncMap::findStepIndex(int id, const vector<vector<int>> &traversable_lanes, int &step_index){

    int current_index = 0;
    for (vector<int> lane_ids: traversable_lanes){
        for (int lane_id: lane_ids){
            if (id == lane_id){
                step_index = current_index;
                return true;
            }
        }
        current_index++;
    }

    return false;
}


// IF YOU WANT TO ADD THE CASE WHERE TWO LANES IN PARALLEL CONVERGES INTO ONE, THEN YOU NEED TO CHANGE THE LOGIC HERE.
bool PncMap::getIntersectionList(int id, double current_forward_s, double current_length, const vector<vector<int>> &traversable_lanes,
                         int step_index, int &count, vector<vector<int>> &list_intersection, int &break_index){
   vector<int> intersection;  // list of lane_ids
   int current_step_index = step_index, currentID = id;
   double forward_s = current_forward_s, length = current_length;
   while (forward_s > length){

       intersection.clear();
       forward_s -= length;

       // check if it's near destination.
       if (traversable_lanes.size() - 1 == current_step_index)   // (ASSIGN NEAR_DESTINATNION)?
            return false;

       // calculate intersection and append to list_intersection
       calIntersection(base_map_->lanes[currentID - 1].front_lane_ids, traversable_lanes[++current_step_index], intersection);
       list_intersection.push_back(intersection);

       if (intersection.size() > 1){
           count = intersection.size();
           break_index = list_intersection.size() - 1;
           break;
       }
       else if (intersection.size() <= 0){
           break_index = list_intersection.size() - 1;
           break;
       }
       else
            currentID = intersection[0];
       length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
   } // end while loop

   break_index = list_intersection.size() - 1;

   return true;
}


void PncMap::calIntersection(const vector<int> &first_list, const vector<int> &second_list, vector<int> &intersection){
    for (int element1: first_list){
        for (int element2: second_list){
            if (element1 == element2)
                intersection.push_back(element1);
        }
    }
}


bool PncMap::haveIntersection(const vector<int> &first_list, const vector<int> &second_list){
    for (int element1: first_list){
        for (int element2: second_list){
            if (element1 == element2)
                return true;
        }
    }
    return false;
}


// -- within the last lane of route.lanes_id, extend as much as you can.
                      //    -- record number of points you added, then delete the same number of points in the front.
void PncMap::finalExtendShifts(int route_id, vector<double> &shifts, vector<int> &lane_IDs, vector<MapPoint> &points){

    cout << "Inside finalExtendShifts. " << endl;

    // extend shifts and points
    int count = 0, lane_index = lane_IDs.back() - 1;
    double length = base_map_->lanes[lane_index].central_points[base_map_->lanes[lane_IDs.back() - 1].central_points_num - 1].s;
    for (double s = shifts.back() + config_param_.STEP_SIZE; s <= length + 0.00001; s += config_param_.STEP_SIZE){
        shifts.push_back(s);
        points.push_back(getPointFromShift(s, lane_index));
        count++;
    }

    // erase first few points
    points.erase(points.begin(), points.begin() + count);

    // (*) check if there's need to erase the first few lane_IDs
    int j = 0;
    for(int k = 0; k < count; k++){
        if ((*shifts_)[route_id].at(k+1) < (*shifts_)[route_id].at(k)){
            j++;
        }
    }
    if (j > 0)
        lane_IDs.erase(lane_IDs.begin(), lane_IDs.begin() + j);

    // erase first few shifts
    shifts.erase(shifts.begin(), shifts.begin() + count);
    (*shifts_)[route_id] = shifts;

    cout << "Outside finalExtendShifts. " << endl;

}


bool PncMap::extendShiftsBackwards(vector<double> &shifts, vector<int> &lane_IDs){

        double backward_s = shifts.front() - config_param_.UPDATE_DISTANCE_BACK;
        int currentID = lane_IDs.front();
        double s=0;
        double flag = shifts.front() - config_param_.STEP_SIZE;
        double length=0;
        int flag_end_lane = 0;

//        cout << "Inside function -- extendShiftsBack. " << endl;

       /// for points backwards
        while (backward_s < 0){
            // get the shifts of the points on this lane
            for(s = flag; s >= 0. - 0.001; s -= config_param_.STEP_SIZE)
                shifts.insert(shifts.begin(), s);
            // get the behind lane id and append in the front of the list
            if (base_map_->lanes[currentID - 1].num_behind_lanes <= 0){
                flag_end_lane = 1;
                break;
            }
            // update currentID
            currentID = base_map_->lanes[currentID - 1].behind_lane_ids[0];
            if (currentID != lane_IDs.front())
                lane_IDs.insert(lane_IDs.begin(), currentID);
            // update length
            length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
            // add the length of the behind lane to backward_s
            backward_s += length;
            // change flag(end) to be inside the next lane
            flag = s + length;
        } // end while loop

        // we are in the case where forward_s < length of the lane, now we need to obtain the last bits for the list
        if(flag_end_lane)
            return false;
        else{
            for (s = flag; s >= backward_s - 0.0001; s -= config_param_.STEP_SIZE)
                shifts.insert(shifts.begin(), s);

            return true;
        }

}


// Problem1--two cases: 1. no more front_lane because of terminal, you should give the same result;
//            2. no more front-lane because of traffic law constraints, you should delete the route.
// approach: another parameter --near_terminal is passed in to be assigned value to indicate if it's case 1 or 2.

// Problem2--this may bring more routes.
// approach: change the parameters into list_shifts and list_laneIDs.

// Advice 1 -- you can refer to getShifts with the global path
bool PncMap::extendShifts(const vector<double> &shifts, const vector<int> &lane_IDs,
                          const vector<vector<int>> &traversable_lanes,
                          vector<vector<double>> &list_shifts, vector<vector<int>> &list_lane_IDs,
                          bool &near_terminal, bool &near_destination){

    double forward_s = shifts.back() + config_param_.UPDATE_DISTANCE;
    int currentID = lane_IDs.back();
    double s=0;
    double flag = shifts.back();
    double length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
    vector<double> this_shifts = shifts;
    vector<int> this_lane_IDs = lane_IDs;


    // 0. FIND the current_INDEX (IN FIRST DIMENSION) of "ids" in traversable_lanes
    int step_index;
     if (!findStepIndex(lane_IDs.back(), traversable_lanes, step_index)){
          cout << "Error: current id is not in traversable_lanes according to global path. " << endl;
          return false;
     }

    // 1. get the maximum size - "count" of INTERSECTION between front_lane_ids and the lane_ids in each segment of traversable_lanes covered by length_of(forward+backward)
    //        and the INTERSECTION_LIST
    int count = 1, break_index = 0;
    vector<vector<int>> list_intersection;
    vector<int> intersection;

    calIntersection(traversable_lanes.back(), lane_IDs, intersection);
//    cout << "back of traversable_lanes: " << endl;
//    for (int i = 0; i < traversable_lanes.back().size(); i++)
//        cout << traversable_lanes.back()[i] << " ";
//    cout << endl;

    // THE FIRST CONDITION CAN BE ERASED
//    if (intersection.size() > 0) cout << "1---------------1" << endl;
    if (//intersection.size() > 0 ||
            !getIntersectionList(lane_IDs.back(), forward_s, length, traversable_lanes, step_index, count, list_intersection, break_index)){
        near_destination = true;
        cout << "Near destination. " << endl;
        return false;
    }

    // 2. collect shifts
    int list_intersection_index;
    for (int index = 0; index < count; index++){
        // reset loop variants
        forward_s = shifts.back() + config_param_.UPDATE_DISTANCE;
        currentID = lane_IDs.back();
        length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
        flag = shifts.back();
        this_lane_IDs = lane_IDs;
        this_shifts = shifts;

        // collect data
        list_intersection_index = 0;
        while (forward_s > length){
            // get the shifts of the points on this lane
            for (s = flag + config_param_.STEP_SIZE; s <= length; s += config_param_.STEP_SIZE)
                this_shifts.push_back(s);
            // change the value of forward_s
            forward_s -= length;
            // change flag(for start)
            flag = s - config_param_.STEP_SIZE - length;
            // change the forward lane id and append at the back of the list
            if (base_map_->lanes[currentID - 1].num_front_lanes <= 0){
                near_terminal = true;
                cout << "Near terminal. " << endl;
                return false;
            }
            if (list_intersection_index <= break_index && list_intersection[list_intersection_index].size() <= 0){


                cout << "Something caused 'list_intersection_index <= break_index && "
                        "list_intersection[list_intersection_index].size() <= 0'" << endl;

                return false;   // THERE MIGHT BE SOMETHING WRONG HERE FOR SOME CASE.-- DOUBLE CHECK HERE LATER.
            }
            else if (list_intersection_index > break_index)    // we assume one front_lane for each lane after the divarication
                currentID = base_map_->lanes[currentID - 1].front_lane_ids[0];
            else if (list_intersection[list_intersection_index].size() > 1)
                currentID = list_intersection[list_intersection_index][index];
            else
                currentID = list_intersection[list_intersection_index][0];
            this_lane_IDs.push_back(currentID);
            // change loop variants
            length = base_map_->lanes[currentID - 1].central_points[base_map_->lanes[currentID - 1].central_points_num - 1].s;
            list_intersection_index++;
        } // end while loop

        // collect the last bits for the list and append to list of shifts/lane_IDs
        for (s = flag + config_param_.STEP_SIZE; s <= forward_s + 0.0001; s += config_param_.STEP_SIZE)
            this_shifts.push_back(s);
        list_shifts.push_back(this_shifts);
        list_lane_IDs.push_back(this_lane_IDs);

    } // end for loop

    return true;
}



/// THERE'S FRONT_LANE_IDS[0] USED HERE, THINK ABOUT WHETHER AND HOW YOU NEED TO CHANGE IT
void PncMap::getNeighborPoints(int laneIndex, int nearest_index, MapPoint &p1, MapPoint &p2){

    cout << "Inside function -- getNeighborPoints. " << endl;

    int id;
    int size = base_map_->lanes[laneIndex].central_points_num;

    // get the point before the nearest map point
    if (nearest_index == 0){
        if (base_map_->lanes[laneIndex].num_behind_lanes < 0){
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

    cout << "Getting outside function -- getNeighborPoints. " << endl;

}


void PncMap::getLeftFrontPointIndex(int pointIndex, int laneIndex, vector<int> &lanes_id, int &frontLaneIndex, int &frontPointIndex){
                                                    // PASSED ANOTHER PARAMETER -- ROUTE.LANES_ID FOR TRAVERSAL
    if (pointIndex == base_map_->lanes[lanes_id[laneIndex] - 1].left_boundary.point_num - 1){
        if (laneIndex == lanes_id.size() - 1){
            frontLaneIndex = laneIndex;
            frontPointIndex = pointIndex;
        }
        else{
            frontLaneIndex = laneIndex + 1;
            frontPointIndex = 1;
        }
    }
    else{
        frontLaneIndex = laneIndex;
        frontPointIndex = pointIndex + 1;
    }
}



void PncMap::getRightFrontPointIndex(int pointIndex, int laneIndex, vector<int> &lanes_id, int &frontLaneIndex, int &frontPointIndex){

    if (pointIndex == base_map_->lanes[lanes_id[laneIndex] - 1].left_boundary.point_num - 1){
        if (laneIndex == lanes_id.size() - 1){
            frontLaneIndex = laneIndex;
            frontPointIndex = pointIndex;
        }
        else{
            frontLaneIndex = laneIndex + 1;
            frontPointIndex = 1;
        }
    }
    else{
        frontLaneIndex = laneIndex;
        frontPointIndex = pointIndex + 1;
    }
}


// CHECK HERE
MapPoint PncMap::getPointFromShift(double s, int laneIndex){

    cout << "Inside getPointFromShift. " << endl;

    MapPoint p;

    if (laneIndex < 0 || laneIndex >= base_map_->lanes_num){
        cout << "Error: In PncMap-getPointFromShift, argument-laneIndex passed in is: " << laneIndex << ", which is out of range"<< endl;
        return p;
    }

    int size =  base_map_->lanes[laneIndex].central_points_num;
    int index;

    // check if s is a valid s (within the range of [0 , length of the route])
    if (s < 0. - 0.0000001 || s > base_map_->lanes[laneIndex].central_points[size - 1].s){
        cout << "Error: In PncMap-getPointFromShift, argument-s passed in is: " << s << ", which is out of range"<< endl;
        cout << "And laneIndex passed in is:  "  << laneIndex << endl;
        cout << "length of the lane is: " << base_map_->lanes[laneIndex].central_points[size - 1].s << endl;
        return p;   //// This might cause problem (random value of attributes of p)
    }

    // declare two neighbor map points
    MapPoint p1;
    MapPoint p2;

    // get the nearest forward point and the nearst backward point
    int nearest_index;

    nearest_index = binarySearch(0, size - 1, s, base_map_->lanes[laneIndex].central_points);

    getNeighborPoints(laneIndex, nearest_index, p1, p2);

    if (nearest_index == 0 && base_map_->lanes[laneIndex].num_behind_lanes > 0)
        index = base_map_->lanes[laneIndex].behind_lane_ids[0] - 1;
    else
        index = laneIndex;

    // interpolate using Linear Approximation -- index goes with p1
    p = interpolateUsingLinearApproximation(p1, p2, s, index);

    cout << "Outside getPointFromShift. " << endl;

    return p;
}


Point3D PncMap::getPoint3DFromShift(double s, int laneIndex){

    Point3D p;

    if (laneIndex < 0 || laneIndex >= base_map_->lanes_num){
        cout << "Error: In PncMap-getPoint3DFromShift, argument-laneIndex passed in is: " << laneIndex << ", which is out of range"<< endl;
        return p;
    }

    int size =  base_map_->lanes[laneIndex].central_points_num;
    int index;

    // check if s is a valid s (within the range of [0 , length of the route])
    if (s < 0 || s > base_map_->lanes[laneIndex].central_points[size - 1].s){
        cout << "Error: In PncMap-getPoint3DFromShift, argument-s passed in is: " << s << ", which is out of range"<< endl;
        return p;
    }

    // declare three map points
    MapPoint p1;
    MapPoint p2;

    // get the nearest forward point and the nearst backward point
    int nearest_index;

    nearest_index = binarySearch(0, size - 1, s, base_map_->lanes[laneIndex].central_points);

    if (nearest_index == 0 && base_map_->lanes[laneIndex].num_behind_lanes > 0)
        index = base_map_->lanes[laneIndex].behind_lane_ids[0] - 1;
    else
        index = laneIndex;

    if (base_map_->lanes[laneIndex].central_points[nearest_index].s == s){
        p.x = base_map_->lanes[laneIndex].central_points[nearest_index].point_enu.x;
        p.y = base_map_->lanes[laneIndex].central_points[nearest_index].point_enu.y;
        return p;
    }

    getNeighborPoints(laneIndex, nearest_index, p1, p2);

    // interpolate using Linear Approximation
    p = interpolate3D(p1, p2, s, index);

    return p;
}


double PncMap::calculateDistance(const Point3D &target, MapPoint p){

    return sqrt((target.x - p.point_enu.x)*(target.x - p.point_enu.x) + (target.y - p.point_enu.y)*(target.y - p.point_enu.y));
}


double PncMap::calDistance(Point3D position, MapPoint p){
    return sqrt((position.x - p.point_enu.x)*(position.x - p.point_enu.x) + (position.y - p.point_enu.y)*(position.y - p.point_enu.y));
}

double PncMap::calDistance(Point3D position, Point3D p){
    return sqrt((position.x - p.x)*(position.x - p.x) + (position.y - p.y)*(position.y - p.y));
}

double PncMap::calDis(MapPoint &p1, MapPoint &p2){
    return sqrt((p1.point_enu.x - p2.point_enu.x)*(p1.point_enu.x - p2.point_enu.x) + (p1.point_enu.y -  p2.point_enu.y)*(p1.point_enu.y -  p2.point_enu.y));
}

double PncMap::calDisToLine(Point3D p, MapPoint p1, MapPoint p2){
    return fabs((p.x - p1.point_enu.x) * (p2.point_enu.y - p1.point_enu.y) - (p2.point_enu.x - p1.point_enu.x) * (p.y - p1.point_enu.y))
            / sqrt((p2.point_enu.x - p1.point_enu.x) * (p2.point_enu.x - p1.point_enu.x) + (p2.point_enu.y - p1.point_enu.y)*(p2.point_enu.y - p1.point_enu.y));
}

double PncMap::calDisToLine(Point3D p, Point3D p1, Point3D p2){
    return fabs((p.x - p1.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p.y - p1.y))
            / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
}

double PncMap::calDisToLine(MapPoint p, Point3D p1, Point3D p2){
    return fabs((p.point_enu.x - p1.x) * (p2.y - p1.y) - (p2.x - p1.x) * (p.point_enu.y - p1.y))
            / sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
}

// by doing vector dot product
pair<int, double> PncMap::getProjectedShift(int laneIndex, int nearest_index, MapPoint p1, MapPoint p2, Point3D vs){

    double s1, s2, s, length;
    int laneIndex_p1, laneIndex_p;
    int size = base_map_->lanes[laneIndex].central_points_num;

    s1 = p1.s;

    // border case 1
    if(nearest_index == 0){

        if (base_map_->lanes[laneIndex].num_behind_lanes <= 0)
            return pair<int, double>(laneIndex + 1, 0.);

        laneIndex_p1 = base_map_->lanes[laneIndex].behind_lane_ids[0] - 1;
        length = base_map_->lanes[laneIndex_p1].central_points[base_map_->lanes[laneIndex_p1].central_points_num - 1].s;
        s2 = p2.s + length;
        s = s1 + ((vs.x - p1.point_enu.x) * (p2.point_enu.x - p1.point_enu.x) + (vs.y - p1.point_enu.y) * (p2.point_enu.y - p1.point_enu.y)) / (s2 - s1);
        if (s > length){
            s -= length;
            laneIndex_p = laneIndex;
        }
        else
            laneIndex_p = laneIndex_p1;
    }
    // border case 2
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

    return pair<int, double>(laneIndex_p + 1, s);
}


int PncMap::binarySearch(int low, int high, double key, const vector<MapPoint> &a){
    if (a.size() <= 0)
        cout << "In function-binarySearch, the argument 'a' entered is wrong. " << endl;

    while(low < high){
        int mid = (low + high)/2;
        if(a[mid].s < key){
            if (a[mid + 1].s > key){
                if ((key - a[mid].s) > (a[mid + 1].s -key))
                    return mid+1;
                else
                    return mid;
            }
            else
                low = mid + 1;
        }
        else if(a[mid].s > key)
            high = mid - 1;
        else
            return mid;
    } // end while loop
    return low;
}



// laneIndex goes with p1
MapPoint PncMap::interpolateUsingLinearApproximation(MapPoint &p1, MapPoint &p2, double s, int laneIndex) {

    // declare a variable to represent the shift of p2.s
    double s_p2;
    // declare another one to represent the shift of p.s
    double s_p;

    // for the case where p1 and p2 are in different lanes:
    if (p1.s > p2.s){
        // make p1.s and p2.s continues
        double length = base_map_->lanes[laneIndex].central_points[base_map_->lanes[laneIndex].central_points_num - 1].s;   // length of the lane where p1 is in
        s_p2 = p2.s + length;
        if (s > p1.s)
            s_p = s;
        else
            s_p = s + length;
    }
    else{
        s_p2 = p2.s;
        s_p = s;
    }

    // ................ calculate attributes ...........................
    MapPoint p;
    double yaw, new_yaw;
    double weight;

    weight = (s_p - s_p2) / (p1.s - s_p2);

    p.s = s;
    p.point_enu.x = (1 - weight) * p2.point_enu.x + weight * p1.point_enu.x;
    p.point_enu.y = (1 - weight) * p2.point_enu.y + weight * p1.point_enu.y;
    p.point_enu.z = (1 - weight) * p2.point_enu.z + weight * p1.point_enu.z;
    p.point_llh.lon = (1 - weight) * p2.point_llh.lon + weight * p1.point_llh.lon;
    p.point_llh.lat = (1 - weight) * p2.point_llh.lat + weight * p1.point_llh.lat;
    p.point_llh.height = (1 - weight) * p2.point_llh.height + weight * p1.point_llh.height;

    /// Make sure you add this bit when you use pitch and roll
    if (p1.euler_angles.yaw * p2.euler_angles.yaw > 0 || fabs(p1.euler_angles.yaw - p2.euler_angles.yaw) < 180)
        p.euler_angles.yaw = (1 - weight) * p2.euler_angles.yaw + weight * p1.euler_angles.yaw;
    else{
        if (p1.euler_angles.yaw < 0){
            yaw = p1.euler_angles.yaw + 360;
            new_yaw = (1 - weight) * p2.euler_angles.yaw + weight * yaw;
            if(new_yaw <= 180)
                p.euler_angles.yaw = new_yaw;
            else
                p.euler_angles.yaw = new_yaw - 360;
        }
        else {
            yaw = p2.euler_angles.yaw + 360;
            new_yaw = (1 - weight) * yaw + weight * p1.euler_angles.yaw;
            if(new_yaw <= 180)
                p.euler_angles.yaw = new_yaw;
            else
                p.euler_angles.yaw = new_yaw - 360;
        }
    }

    // interpolate curvature
    p.kappa = (1 - weight) * p2.kappa + weight * p1.kappa;

    return p;
}



// s is the shift "on lane" for(of) the point to be interpolated
Point3D PncMap::interpolate3D(MapPoint &p1, MapPoint &p2, double s, int laneIndex) {

    Point3D p;

    // declare a variable to represent the shift of p2.s
    double s_p2;
    // declare another one to represent the shift of p.s
    double s_p;

    // for the case where p1 and p2 are in different lanes:
    if (p1.s > p2.s){
        // make p1.s and p2.s continues
        double length = base_map_->lanes[laneIndex].central_points[base_map_->lanes[laneIndex].central_points_num - 1].s;   // length of the lane where p1 is in
        s_p2 = p2.s + length;
        if (s > p1.s)
            s_p = s;
        else
            s_p = s + length;
    }
    else{
        s_p2 = p2.s;
        s_p = s;
    }

    double weight = (s_p - s_p2) / (p1.s - s_p2);
    p.x = (1 - weight) * p2.point_enu.x + weight * p1.point_enu.x;
    p.y = (1 - weight) * p2.point_enu.y + weight * p1.point_enu.y;
    p.z = (1 - weight) * p2.point_enu.z + weight * p1.point_enu.z;

    return p;
}



//Point3D PncMap::interpolate3D(MapPoint &p1, MapPoint &p2, double s){
//    Point3D p;

//    double weight = (s - p2.s) / (p1.s - p2.s);
//    p.x = (1 - weight) * p2.point_enu.x + weight * p1.point_enu.x;
//    p.y = (1 - weight) * p2.point_enu.y + weight * p1.point_enu.y;
//    p.z = (1 - weight) * p2.point_enu.z + weight * p1.point_enu.z;

//    return p;
//}


double PncMap::getShiftByInterpolation(MapPoint p1, MapPoint p2, Point3D location){
    double s;
    double weight = calDistance(location, p2) / calDis(p1, p2);
    s = (1 - weight) * p2.s + weight * p1.s;
    return s;
}



} // end namespace
