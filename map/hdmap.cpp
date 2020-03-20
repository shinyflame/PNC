
#include <math.h>
#include <iostream>
#include <iomanip>
#include "hdmap.h"
#include "config/hdmap_config_param_initial.h"
#include "common/get_now_time.h"


namespace hdmap {


HdMap::HdMap(){
    hdmap_common_ = nullptr;
    pnc_map_ = nullptr;
    map_ = nullptr;
}

HdMap::~HdMap(){
    delete hdmap_common_;
    delete pnc_map_;

    // delete every element stored in maps_
    for (vector<const HDMap*> hdmap_pointer_list: maps_){
        for (const HDMap* hdmap_pointer: hdmap_pointer_list)
            delete hdmap_pointer;
    }
}

const HDMap* HdMap::GetMap(){return map_;}  // This map_ is the one that's being used.


void HdMap::GetSwitchedMap(bool &has_switched, int &scene_id, int &map_id){
    has_switched = map_switched_;
    if (map_switched_){     // /////// MAP_SWITCHED_ ASSIGNED TO FALSE ~!!!!!!!!!!!!!!!!!! CHEKCK
        scene_id = current_scene_id_;
        map_id = map_->id;
    }
}

const vector<pair<int,vector<const HDMap*>>>& HdMap::GetAllMap(){  //all maps
      return all_maps_;
}

const string HdMap::GetPncStatusString(){
    auto current_status=PncInformation::Instance()->GetPncStatus();
    auto current_location_status=current_status.file_load_status;
    auto current_projection_status=current_status.projection_status;
    auto current_width_status=current_status.widths_status;
    auto current_route_status=current_status.route_status;
    auto current_intersection_status=current_status.intersection_status;
    auto current_switch_status=current_status.switch_status;
    auto current_final_status=current_status.final_status;

    if(current_location_status!=PncInformation::FileLoadStatus::FILE_LOAD_OK){
        return PncInformation::Instance()->GetFileLoadStatusString();
    }
    else if(current_location_status!=PncInformation::LocationStatus::LOCATION_OK){
        return PncInformation::Instance()->GetLocationStatusString();
    }
    else if(current_projection_status!=PncInformation::ProjectionStatus::PROJECTION_OK){
        return PncInformation::Instance()->GetProjectionStatusString();
    }
    else if(current_width_status!=PncInformation::WidthStatus::Width_OK){
        return PncInformation::Instance()->GetWidthStatusString();
    }
    else if(current_route_status!=PncInformation::RouteStatus::ROUTE_OK){
        return PncInformation::Instance()->GetRouteStatusString();
    }
    else if(current_intersection_status!=PncInformation::IntersectionStatus::INTERSECTION_OK){
        return PncInformation::Instance()->GetIntersectionStatusString();
    }
    else if(current_switch_status==PncInformation::SwitchStatus::SWITCH_ERROR){
        return "SWITCH_ERROR";
    }
    else{
        return "FINAL_OK";
    }
}

/// CHANGE THE FOLLOWING.....................
void HdMap::ParseOverlapLanes(){
    // declare vars
    vector<int> lanes;
    pair<int, int> key, value;
    int first, second;
    char to_int_char_array[10];
    int a = 0, b = 0;

    char *string_content = &((config_param_.OVERLAP_MAP_LANE_MATCHES)[0]);

    if (string_content[0] == '0'){
        cout << "No matches, because there's only one map in the system. " << endl;
        return;
    }
    else {
        int i = 0;
        while (string_content[i+1] != '\0'){
            if (string_content[i] == ':'){
                b = i - 1;
                copy(string_content, a, b, to_int_char_array);
                a = i + 1;
                first = atoi(to_int_char_array);
            }
            else if (string_content[i+1] == '-'){
                b = i;
                copy(string_content, a, b, to_int_char_array);
                a = i + 2;
                second = atoi(to_int_char_array);
                key = pair<int, int>(first, second);
                if (overlap_lanes_.count(first)){
                    overlap_lanes_[first].push_back(second);
                }
                else{
                    lanes.clear();
                    lanes.push_back(second);
                    overlap_lanes_[first] = lanes;
                }
            }
            else if (string_content[i+1] == ';'){
                b = i;
                copy(string_content, a, b, to_int_char_array);
                a = i + 2;
                second = atoi(to_int_char_array);
                value = pair<int, int>(first, second);
                overlap_matches_[key] = value;
                if (overlap_lanes_.count(first)){
                    overlap_lanes_[first].push_back(second);
                }
                else{
                    lanes.clear();
                    lanes.push_back(second);
                    overlap_lanes_[first] = lanes;
                }
            }
            i++;
        }
    }


    /////// show overlap_matches
      cout << "map_lane_matches: " << endl;
        map<pair<int, int>, pair<int, int>>::iterator iter;
        iter = overlap_matches_.begin();
        while(iter != overlap_matches_.end()) {
          cout << iter->first.first << ":" << iter->first.second << " - " ;
          cout << iter->second.first << ":" << iter->second.second << "; " ;
          iter++;
        }
        cout << endl;

}

void HdMap::copy(char* copy_from, int index_start, int index_end, char copy_to[]){
    // copy from index a to b
    for (int i = index_start; i <= index_end; i++){
        copy_to[i - index_start] = copy_from[i];
    }

    // append null-terminated symbol
    copy_to[index_end - index_start + 1] = '\0';
}



bool HdMap::MapInit(string path_map, int scene_id) {
    if (!HDMapUtil::loadMapFromFile(path_map)){
        PncInformation::Instance()->MutableGetPncStatus()->file_load_status=PncInformation::FileLoadStatus::MAP_LOAD_ERROR;
        return false;
}
    cout << "map id: " << HDMapUtil::base_map_->id << endl;


    //////////// show /////////////////////////
    // show range lon and lat
    cout << "range lon and lat: " << endl;
    for (double lon_lat: HDMapUtil::base_map_->range_lon_lat){
        cout << lon_lat << " ";
    }

    cout << endl;

//    if (HDMapUtil::base_map_->id == 2){

//        cout << "map coordinates of first point of lane 2 in map 2: " << endl;

//        cout << "(" << HDMapUtil::base_map_->lanes[1].central_points[0].point_enu.x
//                << ", " <<  HDMapUtil::base_map_->lanes[1].central_points[0].point_enu.y << ")" << endl;

//    }

    cout << "num_inner_lanes: " << HDMapUtil::base_map_->num_inner_lanes << endl;
    cout << "lane ids: " << endl;
    for (Lane lane: HDMapUtil::base_map_->lanes){
        cout << lane.id << " ";
    }
    cout << endl;
    /////////////////////////////////////



    ////// add map with respect of scene ////////
    // Add this map to maps_
    auto temp_size=maps_.size();
    if (scene_id > (maps_.size() + 1)){
        cout << "Wrong input order of the maps!";
         PncInformation::Instance()->MutableGetPncStatus()->file_load_status=PncInformation::FileLoadStatus::MAP_LOAD_ERROR;
        return false;
    }

    if (scene_id > maps_.size()){
        vector<const HDMap*> this_scene_maps;
        this_scene_maps.clear();
        this_scene_maps.push_back(HDMapUtil::BaseMapPtr());
        all_maps_.push_back(make_pair(scene_id,this_scene_maps));
        maps_.push_back(this_scene_maps);
    }
    else if (scene_id == maps_.size()) {
        maps_[scene_id - 1].push_back(HDMapUtil::BaseMapPtr());

        for(auto& temp_pair:all_maps_){
            if(temp_pair.first==scene_id){
                temp_pair.second.push_back(HDMapUtil::BaseMapPtr());
            }
        }
    }

    else{
        cout << "Error: check your input order of the maps! " << endl;
        return false;
    }
    cout<<"///////////////////////"<<endl;
    cout<<"Globally current scene size is "<<maps_.size()<<endl;
    cout<<"Locally in current scene, map size is "<<maps_[scene_id-1].size()<<endl;
    cout<<"///////////////////////"<<endl;
    return true;
}



bool HdMap::ConfigInit(string path_config) {
    init_state_ = true;
    if(!ConfigParamInit(path_config, config_param_)){
         PncInformation::Instance()->MutableGetPncStatus()->file_load_status=PncInformation::FileLoadStatus::CONFIG_LOAD_ERROR;
                return false;
    }

    if(config_param_.DEBUG_STATE==1){
        PncInformation::Instance()->MutableGetPncStatus()->mode=PncInformation::Mode::SIMULATION_MODE;
    }
    else if(config_param_.DEBUG_STATE==0){
        PncInformation::Instance()->MutableGetPncStatus()->mode=PncInformation::Mode::RECORD_MODE;
    }

    // parse config_param_.OVERLAP_MAP_LANE_MATCHES to overlap_lanes_
    ParseOverlapLanes();

    return true;
}


bool HdMap::RunOnce(const PublishData::MSFLoutput &msfl_output,
                    Point3D &destination, vector<int> &global_path,
        TrafficLightGroups &traffic_light_groups, PncRoutes &pnc_routes){
    cout  << "MapRunOnce starts. " << endl;

    cout << "Input msfl_output(lon-lat): (" << setprecision(6)<< msfl_output.lon << ", " <<  setprecision(6)<<msfl_output.lat << ")" << endl;
    cout << "Input msfl_output(yaw): " << msfl_output.yaw << endl;


    if (!init_state_){
        cout << "Map Initialization unfinished! " << endl;
        return false;
    }

    /// THE REASON FOR FOLLOWING SETTING:
    /// TWO CASES: 1. THE RUNONCE IMMEDIATELY AFTER SETMAP SHOULDN'T CHANGE "MAP_SWITCHED",
    ///                 OTHERWISE THE GETSWITCHEDMAP WON'T CATCH THE "MAP_SWITCHED" CAUSED BY SETMAP
    ///            2. THE "MAP_SWITCHED" SET INSIDE RUNONCE, THEN GETSWITCHEDMAP AFTER RUNONCE WILL CATCH THIS.
    if (map_switched_)
        map_switched_ = false;


    Point3D vehicle_map_coordinates;
    vector<int> map_indices;
    bool found = false;
    bool  selected_found =false;
    // clear input
    pnc_routes.clear();
    traffic_light_groups.light_groups.clear();

    auto current_select_again=PncInformation::Instance()->MutableGetInstruction()->select_again;
    auto current_use_instruction=PncInformation::Instance()->MutableGetInstruction()->use_instruction;
    auto current_instruction_scene_id=PncInformation::Instance()->MutableGetInstruction()->selected_scene_id;
    auto current_instruction_map_id=PncInformation::Instance()->MutableGetInstruction()->selected_map_id;
    auto current_instruction_switch=PncInformation::Instance()->MutableGetInstruction()->switch_map;

    cout<<"current use_instruction is "<<current_use_instruction<<endl;
    cout<<"current use_instruction_after_reset is "<<this->use_instruction_after_reset_<<endl;

    //////////////////////// get current vehicle state & check valid map if map unassigned ///////////////////////////

    /////////////////////////////// DEBUG_STATE ////////////////////////////////
    if (config_param_.DEBUG_STATE){

        cout << "In debug state. " << endl;

        ///// assign vehicle state in map coorinate system /////
        vehicle_map_coordinates.x = msfl_output.lon;
        vehicle_map_coordinates.y = msfl_output.lat;
        vehicle_map_coordinates.z = msfl_output.height;

        if (map_assignment_ == 2){  // MAP UNASSIGNED

            if(current_use_instruction || use_instruction_after_reset_){

                 for (int index_map = 0; index_map < maps_[current_instruction_scene_id-1].size(); index_map++){
                     if (vehicle_map_coordinates.x > maps_[current_instruction_scene_id-1][index_map]->range_x_y[0] - 2
                          && vehicle_map_coordinates.x < maps_[current_instruction_scene_id-1][index_map]->range_x_y[1] + 2
                          && vehicle_map_coordinates.y > maps_[current_instruction_scene_id-1][index_map]->range_x_y[2] - 2
                          && vehicle_map_coordinates.y < maps_[current_instruction_scene_id-1][index_map]->range_x_y[3] + 2){
                          map_indices.push_back(index_map);
                          selected_found  = true;
                           use_instruction_after_reset_=false;

                       current_scene_id_ = current_instruction_scene_id;   // index_scene usually is 0
                       PncInformation::Instance()->MutableGetPncStatus()->location_status==PncInformation::LocationStatus::LOCATION_OK;

                    }
                 }
               if(!selected_found){
                    PncInformation::Instance()->MutableGetPncStatus()->location_status=PncInformation::LocationStatus::LOCATION_XYZ_OUT_OF_RANGE;
                   cout << " Not in  valid  selected map. Auto Selected map " << endl;
                }

            }


            if((current_use_instruction && !selected_found) || (!current_use_instruction && !use_instruction_after_reset_)){
            // check if the vehicle's (x, y) is inside some map of some scene.
            for (int index_scene = 0; index_scene < maps_.size(); index_scene++){
                for (int index_map = 0; index_map < maps_[index_scene].size(); index_map++){
                    if (vehicle_map_coordinates.x > maps_[index_scene][index_map]->range_x_y[0] - 2
                            && vehicle_map_coordinates.x < maps_[index_scene][index_map]->range_x_y[1] + 2
                            && vehicle_map_coordinates.y > maps_[index_scene][index_map]->range_x_y[2] - 2
                            && vehicle_map_coordinates.y < maps_[index_scene][index_map]->range_x_y[3] + 2){
                        map_indices.push_back(index_map);
                        found = true;
                    }
                }
                if (found){
                    current_scene_id_ = index_scene + 1;   // index_scene should be 0.
                    PncInformation::Instance()->MutableGetPncStatus()->location_status==PncInformation::LocationStatus::LOCATION_OK;
                    break;
                }
            }

              }

            if (!found&& !selected_found){         // in our simulation test, this won't happen for now
                 PncInformation::Instance()->MutableGetPncStatus()->location_status=PncInformation::LocationStatus::LOCATION_XYZ_OUT_OF_RANGE;
                cout << "Not in any valid scene area. " << endl;
                return false;
            }

            cout << "According to map_indices, map ids to be checked: " << endl;
            for (int index: map_indices){
                cout << index + 1 << " ";
            }
            cout << endl;

        }
        else {          // MAP ASSIGNED -- either by user or self-accommodating program
            // check if vehicle's (x,y) is inside map_ of current scene.
                   if(current_use_instruction){
                       PncInformation::Instance()->MutableGetInstruction()->use_instruction=false;
                       use_instruction_after_reset_=true;
                       Reset();
                    }

            if (!(vehicle_map_coordinates.x > map_->range_x_y[0] - 2
                    && vehicle_map_coordinates.x < map_->range_x_y[1] + 2
                    && vehicle_map_coordinates.y > map_->range_x_y[2] - 2
                    && vehicle_map_coordinates.y < map_->range_x_y[3] + 2)){
                Reset();   // reset everything
                        PncInformation::Instance()->MutableGetPncStatus()->location_status=PncInformation::LocationStatus::LOCATION_XYZ_OUT_OF_RANGE;
                cout << "Vehicle outside current map area, entering self_accommodating mode... "  << endl;
                        //"Manual navigation towards drivable area of the assigned map needs to be done. " << endl;
                return false;
            }
            else{
                PncInformation::Instance()->MutableGetPncStatus()->location_status==PncInformation::LocationStatus::LOCATION_OK;
            }
        }
    } // end DEBUG_STATE

    else{   /////////////////////////// NON_DEBUG_STATE ///////////////////////////////////
        if (map_assignment_ == 2){  // map unassigned

//            if (msfl_output.gpsState != 4 && msfl_output.gpsState != 5){
//                cout << "GPS unstable! " << endl;
//                return false;
//            }

//            if (msfl_output.navState <= 1){
//                cout << "Localization signal unstable: msfl_output.navState <= 1";
//                return false;
//            }

            if (msfl_output.yaw == 0){
                cout << "Invalid yaw from msfl_output! " << endl;
                return false;
            }

            if(current_use_instruction || use_instruction_after_reset_){

                 for (int index_map = 0; index_map < maps_[current_instruction_scene_id-1].size(); index_map++){
                     if (msfl_output.lon > maps_[current_instruction_scene_id-1][index_map]->range_lon_lat[0]
                          && msfl_output.lon < maps_[current_instruction_scene_id-1][index_map]->range_lon_lat[1]
                          && msfl_output.lat > maps_[current_instruction_scene_id-1][index_map]->range_lon_lat[2]
                          && msfl_output.lat < maps_[current_instruction_scene_id-1][index_map]->range_lon_lat[3]){
                          map_indices.push_back(index_map);
                          selected_found  = true;
                          use_instruction_after_reset_=false;

                       current_scene_id_ = current_instruction_scene_id;   // index_scene usually is 0
                       PncInformation::Instance()->MutableGetPncStatus()->location_status==PncInformation::LocationStatus::LOCATION_OK;

                    }
                 }
               if(!selected_found){
                    PncInformation::Instance()->MutableGetPncStatus()->location_status=PncInformation::LocationStatus::LOCATION_XYZ_OUT_OF_RANGE;
                   cout << " Not in  valid  selected map. Auto Selected map " << endl;
                }

            }


            // check if the vehicle's (lon, lat) is inside some map of some scene.
                    // ASSUMING THE LON-LAT-BORDERS ARE GIVEN PRECISE...
                    // I.E., A POINT CAN'T BE IN BOTH ONE AND ANOTHER MAP.



         if((current_use_instruction && !selected_found) || (!current_use_instruction && !use_instruction_after_reset_)){
            for (int index_scene = 0; index_scene < maps_.size(); index_scene++){
                for (int index_map = 0; index_map < maps_[index_scene].size(); index_map++){
                    if (msfl_output.lon >= maps_[index_scene][index_map]->range_lon_lat[0]
                            && msfl_output.lon <= maps_[index_scene][index_map]->range_lon_lat[1]
                            && msfl_output.lat >= maps_[index_scene][index_map]->range_lon_lat[2]
                            && msfl_output.lat <= maps_[index_scene][index_map]->range_lon_lat[3]){
                        map_indices.push_back(index_map);
                        found = true;
                    }
                }
                if (found){
                    current_scene_id_ = index_scene + 1;
                         PncInformation::Instance()->MutableGetPncStatus()->location_status==PncInformation::LocationStatus::LOCATION_OK;
                    break;
                }
            }
         }

            if (!found && !selected_found){
                        PncInformation::Instance()->MutableGetPncStatus()->location_status=PncInformation::LocationStatus::LOCATION_LLH_OUT_OF_RANGE;
                cout << "Vehicle not in any valid scene area. "
                      "Manual navigation towards valid scene area needs to be done. "<< endl;
                return false;
            }

            cout << "According to map_indices, map ids to be checked: " << endl;
            for (int index: map_indices){
                cout << index + 1 << " ";
            }
            cout << endl;

            // set temp map_ (just for its origin - COORDINATE TRANSFORM)
            map_ = maps_[current_scene_id_ - 1][map_indices[0]];
        }
        else {          // MAP ASSIGNED -- either by user or self-accommodating program
            // check if vehicle's (lon, lat) is inside map_ of current scene.
            if(current_use_instruction){
                PncInformation::Instance()->MutableGetInstruction()->use_instruction=false;
                use_instruction_after_reset_=true;
                Reset();
             }

            if (!(msfl_output.lon > map_->range_lon_lat[0]
                    && msfl_output.lon < map_->range_lon_lat[1]
                    && msfl_output.lat > map_->range_lon_lat[2]
                    && msfl_output.lat < map_->range_lon_lat[3])){ 
                Reset();  // reset everything
                        PncInformation::Instance()->MutableGetPncStatus()->location_status=PncInformation::LocationStatus::LOCATION_LLH_OUT_OF_RANGE;
                cout << "Vehicle outside assigned map area, entering self_accommodating mode... " << endl;
                return false;
            }
        }

        ///// Once you have decided the scene to be used (chosen temp map_ for origin) /////
        cout << "map original point: (" << setprecision(15) << map_->original_point.point_llh.lon << ", "
             << setprecision(15) <<  map_->original_point.point_llh.lat << ", "
             << setprecision(15) <<  map_->original_point.point_llh.height << ")" << endl;

        // transform coordinates
        PointLLH vehicle_global_coordinates;
        vehicle_global_coordinates.lon = msfl_output.lon;
        vehicle_global_coordinates.lat = msfl_output.lat;
        vehicle_global_coordinates.height = msfl_output.height;
        vehicle_map_coordinates = HDMapUtil::mapCoordintateTransform(vehicle_global_coordinates);

        // print lon,lat
        cout << "(lon, lat, height): (" << msfl_output.lon << ", " << msfl_output.lat << ", " << msfl_output.height << ")" << endl;

        // print map_coordinates
        cout << "(x,y,z): (" << vehicle_map_coordinates.x << ", " << vehicle_map_coordinates.y << ", " << vehicle_map_coordinates.z << ")" << endl;

    }
      cout<<"Location check finished"<<endl;

    //////// If vehicle inside Drivable Area, Get its projection on lane /////////
    int id;
    double s;

    if (map_assignment_ == 2){  // MAP UNASSIGNED
        found = false;

        /////// find the nearest valid (point,) lane, map //////
        // (1) getNearestValidPoint among valid maps of the scene.
        vector<int> nearest_map_indices, nearest_lane_indices, nearest_point_indices;
        vector<double> distances;
        cout<<"Ready to check and assign map"<<endl;
        double distance;
        int nearest_lane_index, nearest_point_index;
        for (int map_index: map_indices){
            delete hdmap_common_;
            hdmap_common_ = new HDMapCommon(maps_[current_scene_id_ - 1][map_index], config_param_);
            // get nearest point
             if (hdmap_common_->getNearestPoint(vehicle_map_coordinates, distance, nearest_lane_index, nearest_point_index)){
             // distance <=6
                 cout << "Before CheckIsotonicity" << endl;

                 if (CheckIsotonicity(msfl_output.yaw, maps_[current_scene_id_ - 1][map_index]->lanes[nearest_lane_index].central_points[nearest_point_index].euler_angles.yaw)){
                    //////////////////////////////////////////////////////////////////////////
                     found = true;
                     cout<<"Isotonicity succuss"<<endl;
                     distances.push_back(distance);
                     nearest_lane_indices.push_back(nearest_lane_index);
                     nearest_point_indices.push_back(nearest_point_index);
                     nearest_map_indices.push_back(map_index);
                }
            }
        }
        //////////// TWO CASES: FOUND & NOT FOUND ///////////
        if (!found){
            cout << "Vehicle not in drivable area of self-driving mode. ""Manual navigation towards drivable area needs to be done. "<< endl;
            return false;
        }
        else {        // found valid map (in drivable area)
            // (2) compare the distances of the valid nearest points (to vehicle) on lanes of maps,
            //       choose the nearest, and assign map_
             PncInformation::Instance()->MutableGetPncStatus()->projection_status=PncInformation::ProjectionStatus::PROJECTION_OK;

            double min_distance = config_param_.MAX_NUMBER;
            int min_map_index=-1;
            int min_lane_index, min_point_index;

                    //all distance is less than 6
            for (int index = 0; index < distances.size(); index++){

                if(nearest_map_indices[index]>min_map_index){//choose the largest map index
                  if (distances[index] < min_distance){
                    min_map_index = nearest_map_indices[index];
                    min_distance = distances[index];
                    min_lane_index = nearest_lane_indices[index];
                    min_point_index = nearest_point_indices[index];
                }
              }
            }
            cout << "Found min_map_index: " << min_map_index << ", which means map id is: " << min_map_index + 1 << endl;

            // assign map_ (ALSO SET BASE_MAP_ FOR EACH CLASS -- THIS IS THE FIRST TIME INITIATING CLASSES)
            map_ = maps_[current_scene_id_ - 1][min_map_index];
            delete hdmap_common_;
            hdmap_common_ = new HDMapCommon(map_, config_param_);
            delete pnc_map_;
            pnc_map_ = new PncMap(map_, config_param_);
            HDMapUtil::base_map_ = const_cast<HDMap*>(map_);

            // (3) getProjection on lane with nearest lane/point index (for id and s)
            hdmap_common_->getProjectionWithNearestPoint(vehicle_map_coordinates, min_lane_index, min_point_index, id, s);

            // (4) set flags
            map_assignment_ = MAP_ASSIGNED;    // WHEN VEHICLE ENTERS DRIVABLE AREA FROM UNDRIVABLE AREA

            map_switched_ = true;   // map_ from nullptr to some HDMap pointer

            // (5) run function for planning & vision module (NO NEED FOR deviation check)
            /////////// for planning module ///////////
            if (!pnc_map_->GetRoutesWithRouting(vehicle_map_coordinates, destination, global_path, id, s, pnc_routes))
                return false;
            /////////// for vision module ////////////
            // hdmap_common_->getTrafficLightGroups(id, s, traffic_light_groups);  // not in used for now
        }
    }   // end MAP UNASSIGNED

    else {   // map_assignment_ == 1         // MAP ASSIGNED (either by user or self-accomodating program)
        if (hdmap_common_->deviated){   // vehicle deviated perhaps because of human intervention
            if (hdmap_common_->getFirstProjectionOnLane(vehicle_map_coordinates, msfl_output.yaw, id, s)){
                cout << endl << endl << "deviated -> not deviated" << endl << endl;
                delete hdmap_common_;
                hdmap_common_ = new HDMapCommon(map_, config_param_);
                delete pnc_map_;
                pnc_map_ = new PncMap(map_, config_param_);
            }
            else
                return false;  // not in drivable area
        }

        // not deviated
        if (hdmap_common_->getProjection(vehicle_map_coordinates, msfl_output.yaw, id, s)){
            /////////// for planning module ///////////
            if (!pnc_map_->GetRoutesWithRouting(vehicle_map_coordinates, destination, global_path, id, s, pnc_routes)){
                delete pnc_map_;
                pnc_map_ = new PncMap(map_, config_param_);
                cout<<"GetRoutesWithRouting Error"<<endl;
                return false;
            }
            /////////// for vision module ////////////
            //hdmap_common_->getTrafficLightGroups(id, s, traffic_light_groups);  // not in used for now
        }
        else
            return false;
    }


    ////////////// check if it will switch the map /////////////////

    cout << "going to check if it it needs to automatically switch map. " << endl;

    // ...........check swap condition..................
    if (has_switched_ ){
//         PncInformation::Instance()->MutableGetPncStatus()->switch_status=PncInformation::SwitchStatus::SWITCH_OK;
        if (id != flag_lane_id_
                      && id != map_->lanes[flag_lane_id_-1].left_lane_id
                      && id != map_->lanes[flag_lane_id_-1].right_lane_id){
            if( count_automatic_switch_ == 75) {
                cout << "out of the lane where map was switched. Reset has_switched to be false. " << endl;
                    PncInformation::Instance()->MutableGetPncStatus()->switch_status=PncInformation::SwitchStatus::SWITCH_OK;
                // reset
                count_automatic_switch_ = 0;
                has_switched_ = false;
            }
            else {
                count_automatic_switch_++;
            }
        }
    }
    else {  // !has_switched
        if( ContainedIn(id, overlap_lanes_[map_->id])
            && count_automatic_switch_ != 50){   // Reached overlap lane
            cout << " overlap case 1. " << endl;
            count_automatic_switch_++;
            cout << "increasing count.  " << endl;
            PncInformation::Instance()->MutableGetPncStatus()->switch_status=PncInformation::SwitchStatus::SWITCH_READY;
        }

        // condition of switching map
        else if ( ContainedIn(id, overlap_lanes_[map_->id])
                 && count_automatic_switch_ == 50  ){ // Reached overlap lane and ready for map switch

             PncInformation::Instance()->MutableGetPncStatus()->switch_status=PncInformation::SwitchStatus::SWITCH_IN_PROCESS;
            cout << "overlap case 2. " << endl;
            // reset
            count_automatic_switch_ = 0;
            // set
            has_switched_ = true;

            // find the <map,lane> to be switched...
            int new_map_id = 0;
            map<pair<int, int>, pair<int, int>>::iterator iter;
            iter = overlap_matches_.begin();
            while(iter != overlap_matches_.end()) {
                if (iter->first.first == map_->id && iter->first.second == id){

                    new_map_id = iter->second.first;
                    flag_lane_id_ = iter->second.second;   // set flag_lane_id_......

                    break;
                }
//                else if (iter->second.first == map_->id && iter->second.second == id){
//                    new_map_id = iter->first.first;
//                    flag_lane_id_ = iter->first.second;   // set flag_lane_id_......
//                    break;
//                }
                iter++;
            }

      if(CheckMapValidity(new_map_id,maps_[current_scene_id_-1])){
            // change map_ to the other one
            cout << "new_map_id: " << new_map_id << endl;
            cout << "flag_lane_id_: " << flag_lane_id_ << endl;
            cout << "current_scene_id_: " << current_scene_id_ << endl;
            cout << endl << endl << "map changed <from id: " << map_->id << ">"  << endl;


            ///// ERROR HANDLING /////
            if (new_map_id == 0){
                cout << "hdmapconfig file was wrong. Double check it! " << endl;
                return false;
            }
            //////////////////////////

             // ASSIGN MAP_
            ////////////////////map assign in order , not by id

            map_ = maps_[current_scene_id_ - 1][new_map_id - 1]; // data struct vec

            /////// renew HDMapCommon & PncMap ///////
            delete hdmap_common_;
            hdmap_common_ = new HDMapCommon(map_, config_param_);
            delete pnc_map_;
            pnc_map_ = new PncMap(map_, config_param_);
            ///// change base_map of HDMapUtil (for coordinate transform)
            HDMapUtil::base_map_ = const_cast<HDMap*>(map_);

             cout << " <to id: " <<  map_->id << ">" << endl << endl << endl;
        }
            // set flags
            map_switched_ = true;

        } // Reached overlap lane
    } // !has_switched


    cout << "scene id: " << current_scene_id_ << endl;
    cout << "map id: " << map_->id << endl;
     PncInformation::Instance()->MutableGetPncStatus()->scene_id=current_scene_id_;
     PncInformation::Instance()->MutableGetPncStatus()->map_id=map_->id;

    if(PncInformation::Instance()->GetPncStatus().location_status==PncInformation::LocationStatus::LOCATION_OK&&
            PncInformation::Instance()->GetPncStatus().projection_status==PncInformation::ProjectionStatus::PROJECTION_OK&&
            PncInformation::Instance()->GetPncStatus().widths_status==PncInformation::WidthStatus::Width_OK&&
            PncInformation::Instance()->GetPncStatus().route_status==PncInformation::RouteStatus::ROUTE_OK&&
            PncInformation::Instance()->GetPncStatus().intersection_status==PncInformation::IntersectionStatus::INTERSECTION_OK&&
            PncInformation::Instance()->GetPncStatus().switch_status!=PncInformation::SwitchStatus::SWITCH_ERROR
            ){
        PncInformation::Instance()->MutableGetPncStatus()->final_status=PncInformation::FinalStatus::FINAL_OK;
    }
    else{
        PncInformation::Instance()->MutableGetPncStatus()->final_status=PncInformation::FinalStatus::FINAL_ERROR;
    }
    return true;
}


bool HdMap::CheckIsotonicity(double yaw_vehicle, double yaw_map_point){

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
    PncInformation::Instance()->MutableGetPncStatus()->projection_status=PncInformation::ProjectionStatus::PROJECTION_ANGLE_OUT_OF_RANGE;
    return false;
}


void HdMap::Reset(){
    map_assignment_ = MAP_UNASSIGNED;//(1)
    map_switched_ = false; // (3)
    count_automatic_switch_ = 0; //(4)
    has_switched_ = false;
    pnc_map_->SetHistoryStatus(false);
    hdmap_common_->SetHistoryStatus(false);

}


bool HdMap::ContainedIn(int lane_id, vector<int> ids){
    for (int id: ids){
        if (lane_id == id)
            return true;
    }
    return false;
}

bool  HdMap::CheckMapValidity(const int& map_id, vector<const HDMap*> maps){

    for(auto map:maps){
        if(map_id==map->id){
            return true;
        }
    }
    return false;
}


} //namespace hdmap


