#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include<iomanip>
#include "map_interface.h"
#include "config/hdmap_config_param_initial.h"
#include "MSFLPublish.h"


using namespace std;

int main(int argc, char *argv[]){

    cout << "Hello World!" << endl;


    PublishData::MSFLoutput msfl_output;//(114.054428361942, 22.6656314577902, 0);  (-20., 87.)
    msfl_output.lon = -20.;
    msfl_output.lat = 87.;
    hdmap::TrafficLightGroups traffic_light_groups;
    hdmap::PncRoutes pnc_routes;
    hdmap::Point3D destination;
    vector<int> global_path;

   // test map.h
   if (hdmap::MapInit("/home/mengqi/map/hdmap.bin", "/home/mengqi/map/config/config.json")){

       cout << "version: " << hdmap::MapVersion() << endl;

       // set initial global path // 11, 12, 13, 14, 15, 16, 47, 6, 7, 8, 9, 10
        // global_path = {11,12,13,14,15,16,47,6,7,8,9,10};

       // set intial destination
       destination.x = -20.;
       destination.y = 7.5;

       hdmap::MapRunOnce(msfl_output, destination, global_path, traffic_light_groups, pnc_routes);

       /*cout << "side slip: " << pnc_routes[0].side_slip << endl;*/  // UPDATE MAP_STRUCT.H WHEN YOU UPDATE SOLIB

       cout << "is_update: " << pnc_routes[0].is_update << endl;

   }


    return 0;
}
