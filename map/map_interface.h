#ifndef MAP_INTERFACE_
#define MAP_INTERFACE_

#include <vector>
#include <string>
#include "MSFLPublish.h"
#include "HdMapPublish.h"

namespace hdmap {

  extern "C" std::string  MapVersion();


  extern "C" bool MapInit(std::string path_map, int scene_id) ;

  extern "C" bool ConfigInit(std::string path_config);

  extern "C" const vector<pair<int,vector<const HDMap*>>> GetAllMap() ;

  extern "C" const string GetPncStatusString()  ;

  extern "C" const HDMap* GetMap();

  extern "C" void GetSwitchedMap(bool &has_switched, int &scene_id, int &map_id);

  extern "C"  bool SetPncInstruction(const Instruction& Instruction);

  extern "C"  bool GetPncResponse();


  extern "C" bool MapRunOnce(const PublishData::MSFLoutput &msfl_output,
                             Point3D &destination, vector<int> &global_path,
                TrafficLightGroups &traffic_light_groups, PncRoutes &pnc_routes);

}//end namespace
#endif // MAP_INTERFACE_
