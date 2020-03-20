#ifndef HDMAP_H_
#define HDMAP_H_

#include <string>
#include "hdmap/hdmap_util.h"
#include "hdmap/hdmap_common.h"
#include "pnc_map/pnc_map.h"
#include "map_interface.h"


namespace hdmap {

/**
 * @class hdmap
 *
 * @brief hdmap module main class.
 */


class HdMap  {

public:

    HdMap() ;

    ~HdMap();

    static string Version() {return HDMapUtil::version_alg();} //  + "_" + HDMapUtil::version_bin();}

    bool MapInit(string path_map, int scene_id);

    bool ConfigInit(string path_config);

    bool RunOnce(const PublishData::MSFLoutput &msfl_output, Point3D &destination, vector<int> &global_path,
                TrafficLightGroups &traffic_light_groups, PncRoutes &pnc_routes);

    const HDMap* GetDefaultMap();

    const HDMap* GetMap();

    const  vector<pair<int,vector<const HDMap*>>>& GetAllMap();

    const string GetPncStatusString();

    bool SetMap(int scene_id, int map_id);

    void GetSwitchedMap(bool &has_switched, int &scene_id, int &map_id);

    ConfigParam config_param_;

    vector<vector<const HDMap*>> maps_;  // added for "MULTIPLE MAP AUTOMATIC SWITCH"
                                   // FIRST DIMENSION -- index of scene;
                                   // SECOND DIMENSION -- index of map;
    const HDMap* map_ = nullptr;    // this map is the one that's being used

    vector<pair<int,vector<const HDMap*>>> all_maps_;

    bool CheckMapValidity(const int&  map_id,  vector<const HDMap*> maps);

    bool  SetInstruction(const Instruction& instruction){

        PncInformation::Instance()->MutableGetInstruction()->selected_map_id=instruction.selected_map_id;
        PncInformation::Instance()->MutableGetInstruction()->selected_scene_id=instruction.selected_scene_id;
        PncInformation::Instance()->MutableGetInstruction()->select_again=instruction.select_again;
        PncInformation::Instance()->MutableGetInstruction()->switch_map=instruction.switch_map;
        PncInformation::Instance()->MutableGetInstruction()->use_instruction=instruction.use_instruction;
        instruction_=instruction;
    }

    const Instruction& GetInstrcution  () const {return instruction_;}

     const Response& GetResponse  () const {return response_;}

private:

    // PRIVATE FUNCTIONS
    void ParseOverlapLanes();
    void copy(char* copy_from, int index_start, int index_end, char copy_to[]);
    bool CheckIsotonicity(double yaw_vehicle, double yaw_map_point);
    void Reset();
    bool ContainedIn(int lane_id, vector<int> ids);

    // FIELDS
    map<pair<int, int>, pair<int, int>> overlap_matches_; // <<map_id,lane_id>, <map_id,lane_id>>
    map<int, vector<int>> overlap_lanes_;  // <map_id, lane_ids>
    HDMapCommon* hdmap_common_;
    PncMap* pnc_map_;

    // FLAGS
    bool init_state_ = false;
    MapAssignment map_assignment_ = MAP_UNASSIGNED;
    bool map_switched_ = false;
    int count_automatic_switch_ = 0;
    bool has_switched_ = false;
    int flag_lane_id_ = 0;
    int current_scene_id_ = 0;
    int use_instruction_after_reset_=false;
    Instruction instruction_;
    Response response_;

};


} // end namespace hdmap

#endif /* HDMAP_H_ */
