#include <iostream>
#include"map_interface.h"
#include "hdmap.h"
#include "common/serialize_data.h"
#include "common/get_now_time.h"    // CHANGE THIS while union-test

namespace hdmap {

/*
void Save_Map_Input(const PublishData::MSFLoutput msfl_output){

    static double start_time = Clock::NowInSeconds();
    string file_name = "log/data/hdmap/map_input_" + Clock::GetYMDHMS()+".txt" ;
    static ofstream ofs(file_name, ios::out | ios::binary);

    if(Clock::NowInSeconds() - start_time > 600){
        start_time = Clock::NowInSeconds();
        file_name = "log/data/hdmap/map_input_" + Clock::GetYMDHMS()+".txt" ;
        ofs.close();
        ofs.open(file_name, ios::out | ios::binary);
    }

    if(ofs.is_open()){
        cout << "file open ok!!!" << endl;
    } else {
        cout << "file open failed!!!" << endl;
    }	

    boost::archive::binary_oarchive oa(ofs);
    oa << msfl_output;
}
*/

void Save_Map_Input(const PublishData::MSFLoutput msfl_output){

    string file_name = "log/data/hdmap/map_input.txt" ;
    static ofstream ofs(file_name, ios::out | ios::binary);

    if(ofs.is_open()){
        cout << "file open ok!!!" << endl;
    } else {
        cout << "file open failed!!!" << endl;
    }

    boost::archive::binary_oarchive oa(ofs);
    oa << msfl_output;
}


//*********************************************************



HdMap g_map;

std::string  MapVersion()  {return HdMap::Version();}

bool ConfigInit(std::string path_config){return g_map.ConfigInit(path_config);}

bool MapInit(std::string path_map, int scene_id){return g_map.MapInit(path_map, scene_id);}



/*
bool SetMap(int scene_id, int map_id){

    if (scene_id <= 0 || map_id <= 0){
        cout << "Wrong Input scene_id or map_id: both of them should be greater than 0." << endl;
        return false;
    }

    return g_map.SetMap(scene_id, map_id);

}
*/

const HDMap* GetMap() {return g_map.GetMap();}

const vector<pair<int,vector<const HDMap*>>> GetAllMap()  {return g_map.GetAllMap();}

const string GetPncStatusString()  {return g_map.GetPncStatusString();}

bool SetPncInstruction(const Instruction& instruction){

      g_map.SetInstruction(instruction);

}

  bool GetPncResponse();


void GetSwitchedMap(bool &has_switched, int &scene_id, int &map_id){

	g_map.GetSwitchedMap(has_switched, scene_id, map_id);

}


bool MapRunOnce(const PublishData::MSFLoutput &msfl_output, 
		Point3D &destination, vector<int> &global_path,
                TrafficLightGroups &traffic_light_groups,
                PncRoutes &pnc_routes){

  if (!g_map.config_param_.DEBUG_STATE)
      Save_Map_Input(msfl_output);

  return g_map.RunOnce(msfl_output, destination, global_path, traffic_light_groups, pnc_routes);

}


} //namespace planning
