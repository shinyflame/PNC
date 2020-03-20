#include "pnc_information.h"


namespace hdmap {


  BUILD_SHARE_VAR(PncInformation);

//PncInformation::PncStatus PncInformation::pnc_status_;
//PublishData::Instruction PncInformation::instruction_;
Factory<int,int,int*(*)(),std::map<int,int*(*)()>>  PncInformation::pnc_factory;

PncInformation::PncInformation(){
    Init();
    InstructionInit();
    ResponseInit();
}

void PncInformation::Init(){
    pnc_status_.file_load_status=PncInformation::FileLoadStatus::FILE_LOAD_OK;
    pnc_status_.mode=PncInformation::Mode::SIMULATION_MODE;
    pnc_status_.location_status=PncInformation::LocationStatus::LOCATION_OK;
    pnc_status_.projection_status=PncInformation::ProjectionStatus::PROJECTION_OK;
    pnc_status_.widths_status=PncInformation::WidthStatus::Width_OK;
    pnc_status_.route_status=PncInformation::RouteStatus::ROUTE_OK;
    pnc_status_.intersection_status=PncInformation::IntersectionStatus::INTERSECTION_OK;
    pnc_status_.switch_status=PncInformation::SwitchStatus::SWITCH_OK;
    pnc_status_.final_status=PncInformation::FinalStatus::FINAL_ERROR;

}

void PncInformation::InstructionInit(){
    instruction_.use_instruction=false;
    instruction_.selected_scene_id=0;
    instruction_.selected_map_id=0;
    instruction_.switch_map=false;
}

void PncInformation::ResponseInit(){
    response_.current_map_cleaned=false;
    response_.current_scene_cleaned=false;
}

void PncInformation::Clear(){
    pnc_status_={};
}

std::string PncInformation::GetFileLoadStatusString(){

    auto current_status=pnc_status_.file_load_status;
    switch (current_status) {
    case PncInformation::FileLoadStatus::FILE_LOAD_OK: return "FILE_LOAD_OK";break;
    case   PncInformation::FileLoadStatus::MAP_LOAD_ERROR:return "MAP_LOAD_ERROR";break;
    case   PncInformation::FileLoadStatus::CONFIG_LOAD_ERROR:return "CONFIG_LOAD_ERROR";break;
    case   PncInformation::FileLoadStatus::RECORD_LOAD_ERROR:return "RECORD_LOAD_ERROR";break;
    default:return "FILE_LOAD_OK";break;
    }
}

std::string PncInformation::GetModeString(){

    auto current_status=pnc_status_.mode;
    switch (current_status) {
    case PncInformation::Mode::SIMULATION_MODE: return "SIMULATION_MODE";break;
    case   PncInformation::Mode::RECORD_MODE:return "RECORD_MODE";break;
    default:return "SIMULATION_MODE";break;
    }
}

std::string PncInformation::GetLocationStatusString(){
    auto current_status=pnc_status_.location_status;
    switch (current_status) {
    case PncInformation::LocationStatus::LOCATION_OK: return "LOCATION_OK";break;
    case PncInformation::LocationStatus::LOCATION_XYZ_OUT_OF_RANGE:return "LOCATION_XYZ_OUT_OF_RANGE";break;
    case PncInformation::LocationStatus::LOCATION_LLH_OUT_OF_RANGE:return "LOCATION_LLH_OUT_OF_RANGE";break;
    case PncInformation::LocationStatus::LOCATION_ANGLE_OUT_OF_RANGE:return "LOCATION_ANGLE_OUT_OF_RANGE";break;
        default:return "LOCATION_OK";break;
    }

}

std::string PncInformation::GetProjectionStatusString(){
    auto current_status=pnc_status_.projection_status;
    switch (current_status) {
    case PncInformation::ProjectionStatus::PROJECTION_OK: return "PROJECTION_OK";break;
    case PncInformation::ProjectionStatus::PROJECTION_DISTANCE_OUT_OF_RANGE:return"PROJECTION_DISTANCE_OUT_OF_RANGE";break;
    case PncInformation::ProjectionStatus::PROJECTION_ANGLE_OUT_OF_RANGE:return "PROJECTION_ANGLE_OUT_OF_RANGE";break;
        default:return "PROJECTION_OK";break;
    }

}

std::string PncInformation::GetWidthStatusString(){
    auto current_status=pnc_status_.widths_status;
    switch (current_status) {
    case PncInformation::WidthStatus::Width_OK:return "Width_OK";break;
    case PncInformation::WidthStatus::LANE_LEFT_WIDTHS_OUT_OF_RANGE:return"LANE_LEFT_WIDTHS_OUT_OF_RANGE";break;
    case PncInformation::WidthStatus::LANE_RIGHT_WIDTHS_OUT_OF_RANGE :return"LANE_RIGHT_WIDTHS_OUT_OF_RANGE";break;
    case PncInformation::WidthStatus::LANE_LEFT_WIDTHS_MATCH_ERROR:return"LANE_LEFT_WIDTHS_MATCH_ERROR";break;
    case PncInformation::WidthStatus::LANE_RIGHT_WIDTHS_MATCH_ERROR:return"LANE_RIGHT_WIDTHS_MATCH_ERROR";break;
    case PncInformation::WidthStatus::ROAD_WIDTHS_OUT_OF_RANGE:return"ROAD_WIDTHS_OUT_OF_RANGE";break;
        default:return "Width_OK";break;
    }

}

std::string PncInformation::GetRouteStatusString(){
    auto current_status=pnc_status_.route_status;
    switch (current_status) {
    case PncInformation::RouteStatus::ROUTE_OK:return"ROUTE_OK";break;
         case PncInformation::RouteStatus::LANE_CENTER_POINTS_ERROR:return"LANE_CENTER_POINTS_ERROR";break;
         case PncInformation::RouteStatus::LANE_STATION_ERROR:return"LANE_STATION_ERROR";break;
         case PncInformation::RouteStatus::LANE_TERMINAL_ERROR:return"LANE_TERMINAL_ERROR";break;
         case PncInformation::RouteStatus::LANE_SITE_ERROR:return"LANE_SITE_ERROR";break;
        default:return "ROUTE_OK";break;
    }
}

std::string PncInformation::GetIntersectionStatusString(){
    auto current_status=pnc_status_.intersection_status;
    switch (current_status) {
    case PncInformation::IntersectionStatus::INTERSECTION_OK:return"INTERSECTION_OK";break;
    case PncInformation::IntersectionStatus::ONE_TO_MANY_ERROR:return"ONE_TO_MANY_ERROR";break;
    case PncInformation::IntersectionStatus::MANY_TO_ONE_ERROR:return"MANY_TO_ONE_ERROR";break;
        default:return "INTERSECTION_OK";break;
    }
}

std::string PncInformation::GetSwitchStatusString(){
    auto current_status=pnc_status_.switch_status;
    switch (current_status) {
    case PncInformation::SwitchStatus::SWITCH_OK:return"SWITCH_OK";break;
    case PncInformation::SwitchStatus::SWITCH_READY:return"SWITCH_READY";break;
        case PncInformation::SwitchStatus::SWITCH_IN_PROCESS:return"SWITCH_IN_PROCESS";break;
        case PncInformation::SwitchStatus::SWITCH_DONE:return"SWITCH_DONE";break;
        case PncInformation::SwitchStatus::SWITCH_ERROR:return"SWITCH_ERROR";break;
        default:return "SWITCH_OK";break;
    }
}

 std::string PncInformation::GetFinalStatusString(){
     auto current_status=pnc_status_.final_status;
     switch (current_status) {
     case PncInformation::FinalStatus::FINAL_OK:return"FINAL_OK";break;
     case PncInformation::FinalStatus::FINAL_ERROR:return"FINAL_ERROR";break;
         default:return "FINAL_OK";break;
     }

 }


}
