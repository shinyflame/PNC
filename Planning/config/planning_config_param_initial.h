

#ifndef PLANNING_CONFIG_PARAM_INITIAL_H
#define PLANNING_CONFIG_PARAM_INITIAL_H

#include <string>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "../json/json.h"
#include "../common/struct.h"

using namespace std;

namespace planning {


bool ReadObstacles(string flie_path,
                   prediction::PredictionObstacles & prediction_obstacles);
bool ReadObstacles(prediction::PredictionObstacles & prediction_obstacles);

bool ReadTrajectory(string flie_path,
                    std::vector<TrajectoryPoint>& trajectory_points);
bool ReadTrajectory(std::vector<TrajectoryPoint>& trajectory_points);

bool ReadFile(string flie_path,ConfigParam & planing_config);
bool ReadFile(string flie_path,VehicleParam  &vehicle_param);
bool ReadData(int    &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式int
bool ReadData(bool   &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式bool
bool ReadData(double &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式double
bool ReadData(string &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式stirng
bool ReadData(uint32_t &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式uint32
bool ConfigParamInit (string config_path,ConfigParam   &planing_config );
bool VehicleParamInit(string vehicle_param_path,VehicleParam  &vehicle_param);


} //end namespace planning

#endif // PLANNING_CONFIG_PARAM_INITIAL_H
