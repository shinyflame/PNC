#ifndef PNC_INFORMATION_H
#define PNC_INFORMATION_H

#include "pnc_macro.h"
#include "factory.h"
#include "../HdMapPublish.h"

#include <string>
#include<vector>

namespace hdmap {


class PncInformation
{
public:

    enum FileLoadStatus{
        FILE_LOAD_OK=1,
        MAP_LOAD_ERROR=2,
        CONFIG_LOAD_ERROR=3,
        RECORD_LOAD_ERROR
    };

    enum Mode{
        SIMULATION_MODE=1,
        RECORD_MODE=2
    };
    enum LocationStatus{
        LOCATION_OK=1,
        LOCATION_XYZ_OUT_OF_RANGE=2,
        LOCATION_LLH_OUT_OF_RANGE=3,
        LOCATION_ANGLE_OUT_OF_RANGE=4
    };
    enum ProjectionStatus{
        PROJECTION_OK=1,
        PROJECTION_DISTANCE_OUT_OF_RANGE=2,
        PROJECTION_ANGLE_OUT_OF_RANGE=3
    };
    enum WidthStatus{
        Width_OK=1,
        LANE_LEFT_WIDTHS_OUT_OF_RANGE=2,
        LANE_RIGHT_WIDTHS_OUT_OF_RANGE=3,
        LANE_LEFT_WIDTHS_MATCH_ERROR=4,
        LANE_RIGHT_WIDTHS_MATCH_ERROR=5,
        ROAD_WIDTHS_OUT_OF_RANGE=6,

    };
    enum RouteStatus{
        ROUTE_OK=1,
        LANE_CENTER_POINTS_ERROR=2,
        LANE_STATION_ERROR=3,
        LANE_TERMINAL_ERROR=4,
        LANE_SITE_ERROR=5
    };
    enum IntersectionStatus{
        INTERSECTION_OK=1,
        ONE_TO_MANY_ERROR=2,
        MANY_TO_ONE_ERROR=3
    };

    enum  SwitchStatus{
        SWITCH_OK=1,
        SWITCH_READY=2,
        SWITCH_IN_PROCESS=3,
        SWITCH_DONE=4,
        SWITCH_ERROR=5
    };
    enum FinalStatus{
        FINAL_OK=1,
        FINAL_ERROR=2
    };

    struct RouteInfo{
        int points_number;
        int route_id;
        bool side_slip;
        std::vector<int> lanes_ids;
    };

    struct PncStatus{
        FileLoadStatus file_load_status;
        Mode  mode;
        LocationStatus location_status;
        ProjectionStatus projection_status;
        WidthStatus widths_status;
        RouteStatus route_status;
        IntersectionStatus intersection_status;
        SwitchStatus switch_status;
        FinalStatus final_status;
        int map_id;
        int scene_id;
        int routes_number;
        std::vector<RouteInfo> routes_info;
    };


    void Clear();

     void Init();

     void InstructionInit();

     void ResponseInit();

     const PncStatus& GetPncStatus(){return pnc_status_;}

     PncStatus* MutableGetPncStatus(){return &pnc_status_;}

     const Instruction& GetInstruction(){return instruction_;}

     Instruction* MutableGetInstruction(){return &instruction_;}

     const Response& GetResponse(){return response_;}

     Response* MutableGetResponse(){return &response_;}

    std::string GetFileLoadStatusString();

    std::string GetModeString();

    std::string GetLocationStatusString();

    std::string GetProjectionStatusString();

    std::string GetWidthStatusString();

    std::string GetRouteStatusString();

    std::string GetIntersectionStatusString();

    std::string GetSwitchStatusString();

     std::string GetFinalStatusString();

private:
     PncStatus pnc_status_;

      Instruction instruction_;

      Response  response_;

    static Factory<int,int,int*(*)(),std::map<int,int*(*)()>>  pnc_factory;

    DECLARE_SINGLETON(PncInformation);

};
}
#endif // PNC_INFORMATION_H
