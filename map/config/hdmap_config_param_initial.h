

#ifndef PLANNING_CONFIG_PARAM_INITIAL_H
#define PLANNING_CONFIG_PARAM_INITIAL_H

#include <string>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include "../HdMapPublish.h"
#include "../json/json.h"

using namespace std;

namespace hdmap {

bool ReadFile(string flie_path, ConfigParam &map_config);
bool ReadData(int    &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式int
bool ReadData(bool   &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式bool
bool ReadData(double &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式double
bool ReadData(string &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式stirng
//bool ReadData(uint32 &data,string name,Json::Value Root); //从文件中读取JSON，一个存储了JSON格式uint32
bool ConfigParamInit (string config_path, ConfigParam &map_config );


} //end namespace hdmap

#endif // PLANNING_CONFIG_PARAM_INITIAL_H
