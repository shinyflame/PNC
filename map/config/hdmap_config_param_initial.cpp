
#include "hdmap_config_param_initial.h"
#include "../common/pnc_information.h"


namespace hdmap {


bool ReadFile(string flie_path, ConfigParam &map_config)
{

  Json::Reader reader;
  Json::Value root;

  //从文件中读取，保证当前文件有test.json文件
  ifstream in(flie_path,ios::binary);
  //in.open("test.json", ios::binary);

  if(!in.is_open()){
     PncInformation::Instance()->MutableGetPncStatus()-> file_load_status=PncInformation::FileLoadStatus::CONFIG_LOAD_ERROR;
     cout << "Error opening config file\n";
     return false;
   }

  if(reader.parse(in,root)){

      //string name = root["name"].asString();
      //读取子节点信息
      if(
         ReadData(map_config.LOOK_FORWARD_DISTANCE, "01.LOOK_FORWARD_DISTANCE",root)&&
         ReadData(map_config.LOOK_BACKWARD_DISTANCE, "02.LOOK_BACKWARD_DISTANCE",root)&&
         ReadData(map_config.STEP_SIZE,
                             "03.STEP_SIZE",root)&&
         ReadData(map_config.MAX_DISTANCE,
                             "04.MAX_DISTANCE",root)&&
         ReadData(map_config.UPDATE_DISTANCE,"05.UPDATE_DISTANCE",root)&&
         ReadData(map_config.UPDATE_DISTANCE_BACK,"06.UPDATE_DISTANCE_BACK",root)&&
         ReadData(map_config.MAX_NUMBER,    "07.MAX_NUMBER",root)&&
         ReadData(map_config.DEBUG_STATE,  "08.DEBUG_STATE",root)&&
         ReadData(map_config.OVERLAP_MAP_LANE_MATCHES, "09.OVERLAP_MAP_LANE_MATCHES",root)
       )
        {
           cout << "Read config file successfully!" << endl;
           in.close();
           return true;
        }else
        {
          cout << "ERROR: Failed reading config file!!!" << endl;
          in.close();
          return false;

        }

    } else {
      cout << "ERROR: Failed to parse config file! \n" << endl;
       in.close();
       return false;
    }


}



bool ReadData(string &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asString();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}


bool ReadData(bool &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asBool();
      if(data)
         cout<<name<<" = "<<"ture"<<endl;
      else
         cout<<name<<" = "<<"false"<<endl;

      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}


bool ReadData(int &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asInt();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}


//bool ReadData(uint32 &data,string name,Json::Value Root)
//{
//  if(!Root[name].isNull())
//   {
//      data =  Root[name].asUInt();
//      cout<<name<<" = "<<data<<endl;
//      return true;
//   }else
//    {
//      cout<<"read "<<name<<" failed"<<endl;
//      return false;
//    }
//}


bool ReadData(double &data,string name,Json::Value Root)
{
  if(!Root[name].isNull())
   {
      data =  Root[name].asDouble();
      cout<<name<<" = "<<data<<endl;
      return true;
   }else
    {
      cout<<"read "<<name<<" failed"<<endl;
      return false;
    }
}



bool ConfigParamInit(string config_path, ConfigParam &map_config )
{
  return ReadFile(config_path, map_config);

}



} //end namespace hdmap
