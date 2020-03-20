
#include "../common/write_read_file.h"
#include "../common/get_now_time.h"
#include "../common/struct.h"
#include <unistd.h>
#include<iomanip>


using namespace std;
#define random(a,b) (((double)rand()/RAND_MAX)*(b-a)+a)


string outfile = "../../../common/planning_data";

namespace planning {


#if 0
int ReadAndWrite(char *ChassisFilePath,
                 char *LocationFilePath,
                 char *TrafficFilePath,
                 char *PeridictionFilePath,
                 char *PbTrajectoryFilePath,
                 char *OutFusionFilePath
                 ){

  FILE* ChResult = fopen(ChassisFilePath, "r");
  if (ChResult == NULL)
  {
          printf("Read Chassis File result file error\n");
          exit(0);
  }

  FILE* PeResult = fopen(PeridictionFilePath, "r");
  if (ChResult == NULL)
  {
          printf("Read Chassis File result file error\n");
          exit(0);
  }

  FILE* LoResult = fopen(LocationFilePath, "r");
  if (LoResult == NULL)
  {
          printf("Read Location File result file error\n");
          exit(0);
  }
  FILE* TrResult = fopen(TrafficFilePath, "r");
  if (TrResult == NULL)
  {
          printf("Read Traffic File result file error\n");
          exit(0);
  }
  FILE* PbResult = fopen(PbTrajectoryFilePath, "r");
  if (TrResult == NULL)
  {
          printf("Read PbTrajectory File result file error\n");
          exit(0);
  }
  FILE* FuResult = fopen(OutFusionFilePath, "w");
  if (TrResult == NULL)
  {
          printf("Read OutFusion File result file error\n");
          exit(0);
  }

#if 0
///********************** Read Chassis Data *************************************//
    canbus::Chassis chassis_;
    int read_times_= 0;
    cout<<"******** Read Chassis Data Begin ***** "<<endl;
    while(ReadChassisData(ChResult, chassis_,read_times_) > 0);
    cout<<"******** Read Chassis Data End   ***** "<<endl<<endl;
    fclose(ChResult);

///*********************** Read Perception *************************************//
    int obstacles_num_ = 0;
    prediction::PredictionObstacles obstacles_ ;
    cout<<"Read Chassis Data Begin ***** "<<endl;
    while(ReadPeridictionData( PeResult,obstacles_ ,obstacles_num_) > 0);
    cout<<"Read Chassis Data End ***** "<<endl<<endl;
    fclose(PeResult);
///*********************** Read Traffic Light Detection ************************//
    int read_traffic_num_ = 0;
    perception::TrafficLightDetection  traffic_detection_;
    cout<<"+++++++++ Read Traffic Data Begin ***** "<<endl;
    while(ReadTrafficData( TrResult,traffic_detection_,read_traffic_num_) > 0);
    cout<<"+++++++++ Read Traffic Data End   ***** "<<endl<<endl;
    fclose(TrResult);
///*********************** Read Localization Estimate **************************//
    int localizatio_num_ = 0;
    localization::LocalizationEstimate localization_;
    cout<<"Read Localization Estimate Data Begin ***** "<<endl;
    while(ReadLocationData( LoResult,localization_,localizatio_num_) > 0);
    cout<<"Read Localization Estimate Data End ***** "<<endl<<endl;
    fclose(LoResult);

///*********************** Read PbTrajectory *********************************//
    PbTrajectory  pb_trajectory;
    int read_trajectory_times = 0;
    cout<<"++++++++++ Read PbTrajectory Data Begin  "<<endl;
    while(ReadPbTrajectoryData(PbResult,pb_trajectory,read_trajectory_times) >0);
    cout<<"size = "<<pb_trajectory.trajectory_points.size()<<endl;
    cout<<"++++++++++ Read PbTrajectory Data End  "<<endl<<endl;

#endif

    PbTrajectory  pb_trajectory;
    int read_pb_frame = 0;
    localization::LocalizationEstimate localization_;
    int localizatio_frame = 0;
    canbus::Chassis chassis_;
    int read_chassis_frame= 0;
    prediction::PredictionObstacles obstacles_ ;
    int obstacles_frame = 0;
    perception::TrafficLightDetection  traffic_detection_;
    int read_traffic_frame = 0;

    int fusion_data_frame = 0;
    while(1){
        cout<<"***********************fusion start *************************"<<endl<<endl;
        if( ReadPbTrajectoryData(PbResult,pb_trajectory,read_pb_frame)<0)
        {
          cout<<"Read PbTrajectory Data finish ! Frame = "<<read_pb_frame<<endl<<endl;
          break; }

        int a;
        while(1)
        {
          a = ReadLocationData( LoResult,localization_,localizatio_frame);
          if ( a < 0 ) break;
          if(localization_.timestamp_ms == pb_trajectory.pb_header.location_timestamp)
            break;
        }
        if ( a < 0 ){
           break;
           cout<<"Read Localization Data finish ! Frame = "<<localizatio_frame<<endl<<endl;
        }

        int b;
        while(1)
        {
          b = ReadChassisData(ChResult, chassis_,read_chassis_frame);
          if ( b < 0 ) break;
          if(chassis_.timestamp_ms == pb_trajectory.pb_header.chassis_timestamp)
            break;
        }
        if ( b < 0 ){
           break;
           cout<<"Read Chassis Data finish ! Frame = "<<read_chassis_frame<<endl<<endl;
        }

        int c;
        while(1)
        {
          c = ReadPeridictionData( PeResult,obstacles_ ,obstacles_frame) ;
          if ( c < 0 ) break;
          if(obstacles_.start_timestamp == pb_trajectory.pb_header.prediction_timestamp)
            break;
        }
        if ( c < 0 ){
           break;
           cout<<"Read Prediction Data finish ! Frame = "<<obstacles_frame<<endl<<endl;
        }

        int d;
        while(1)
        {
          d = ReadTrafficData( TrResult,traffic_detection_,read_traffic_frame);
          if ( d < 0 ) break;
          if(traffic_detection_.camera_timestamp == pb_trajectory.pb_header.camera_timestamp)
            break;
        }
        if ( d < 0 ){
           break;
           cout<<"Read Traffic Data finish ! Frame = "<<read_traffic_frame<<endl<<endl;
        }


        cout<<"plan_timestamp = "<<pb_trajectory.pb_header.plan_timestamp<<endl;
        cout<<"loc_timestamp  = "<<localization_.timestamp_ms<<endl;
        fusion_data_frame++;
        cout<<"fusion_data_frame = "<<fusion_data_frame<<endl<<endl;

      }

    fclose(PbResult);
    fclose(LoResult);
    fclose(ChResult);
    fclose(PeResult);
    fclose(TrResult);

    return 0;

}


int MakeData(char *ChassisFilePath, char *LocationFilePath,
             char *TrafficFilePath, char *PeridictionFilePath ,
             char *PbTrajectoryFilePath )
{

   FILE* ChResult = fopen(ChassisFilePath, "w");
   if (ChResult == NULL)
   {
           printf("open Chassis File result file error\n");
           exit(0);
   }

   FILE* LoResult = fopen(LocationFilePath, "w");
   if (LoResult == NULL)
   {
           printf("open Location File result file error\n");
           exit(0);
   }
   FILE* TrResult = fopen(TrafficFilePath, "w");
   if (TrResult == NULL)
   {
           printf("open Traffic File result file error\n");
           exit(0);
   }
   FILE* PeResult = fopen(PeridictionFilePath, "w");
   if (PeResult == NULL)
   {
           printf("open Peridiction File result file error\n");
           exit(0);
   }
   FILE* PbResult = fopen(PbTrajectoryFilePath, "w");
   if (PeResult == NULL)
   {
           printf("open PbTrajectory File result file error\n");
           exit(0);
   }
#if 1
 int Frame_ = 0;
 int loop_ = 0;

 srand((int)time(0));
 while(Frame_ < 1000)
   {
     usleep(10000);
     uint64 location_time = Clock::NowInMs();
     MakeLocationData( LoResult, location_time ,Frame_);
     if(loop_ >9){
        PbHeader pb_hearder;
        pb_hearder.location_timestamp   = location_time;
        pb_hearder.chassis_timestamp    = location_time -  random(10,99);
        pb_hearder.prediction_timestamp = location_time -  random(0,99);
        pb_hearder.camera_timestamp     = location_time -  random(20,99);
        pb_hearder.plan_timestamp       = location_time -  random(30,99);

        MakeChassisData(ChResult,pb_hearder.chassis_timestamp,Frame_ );
        MakeTrafficData(TrResult,pb_hearder.camera_timestamp,1+Frame_%3,Frame_);
        MakePeridictionData(PeResult,pb_hearder.prediction_timestamp,2+random(0,5),
                              3+random(0,4),random(0,2),20+random(20,30),Frame_ );
        MakeTrajectoryData(PbResult,pb_hearder);
        loop_ =0;
       }

     loop_ ++;
     Frame_++;

   }
 fclose(ChResult);
 fclose(LoResult);
 fclose(TrResult);
 fclose(PeResult);
 fclose(PbResult);
#endif
#if 0
///*************************chassis data write start ****************************//
   int FrameNum1 = 8;
   uint64 Timestamp_ms1 = Clock::NowInMs();
   for(int i = 0 ; i < FrameNum1; i++ )
     MakeChassisData(ChResult,Timestamp_ms1,i );
     fclose(ChResult);
///************************location data write start ***************************//

  for(int i = 0 ; i < FrameNum1 -3; i++ )
    MakeLocationData( LoResult,Timestamp_ms1 ,i);
    fclose(LoResult);

///************************Traffic data write start ***************************//
  int TrafficFram = 8;
  for(int i = 0 ; i < FrameNum1 -3; i++ )
    MakeTrafficData(TrResult,Timestamp_ms1,TrafficFram,i);

   fclose(TrResult);
///***********************Prediction data write start *************************//

    uint64 Timestamp_ms = Clock::NowInMs();
    int32  ObstacleNum = 7;
    int32  PolygonNum = 9;
    int32  TrajectoryNum = 1;
    int32  TrajectoryPointNum = 25;
    int32  FrameNum = 19 ;

    for(int i = 0 ; i < FrameNum ; i ++)
      MakePeridictionData(PeResult,Timestamp_ms,ObstacleNum,
                          PolygonNum,TrajectoryNum,TrajectoryPointNum,i );
    fclose(PeResult);

/// end ***************************//


///***********************PbTrajectory data write start *************************//
    cout<<" "<<endl;
    cout<<"make Trajectory ..."<<endl;
    PbHeader Timestamp;
    Timestamp.plan_timestamp = Clock::NowInMs();
    Timestamp.chassis_timestamp = Clock::NowInMs();
    Timestamp.location_timestamp = Clock::NowInMs();
    Timestamp.prediction_timestamp = Clock::NowInMs();
    Timestamp.camera_timestamp = Clock::NowInMs();

    cout<<"Timestamp = "<<Timestamp.camera_timestamp<<endl;
    int j= 0;
    for(int i = 0 ; i < 5 ; i++){


       MakeTrajectoryData(PbResult,Timestamp,j);
      }
    cout<<"make Trajectory ..."<<endl<<endl;
   fclose(PbResult);
//***********************PbTrajectory data write end *************************//
#endif
    return 0;

}
//**************************************************************************************//
//**************************************************************************************//
//**************************************************************************************//
//**************************************************************************************//

int ReadPbTrajectoryData( FILE* PbResult,
                          PbTrajectory  &pb_trajectory,
                          int &read_times_ ){
  int len = 0;
  char DataBuf[65535] = { 0 };
  char* ptrSit = DataBuf;

  int data_long = sizeof(PbHeader) + sizeof(VehiclePositonState) + 8 ;

  len = fread(DataBuf, sizeof(char), data_long, PbResult);
  if (len != data_long){
      cout<<"Read PbTrajectory Finish !!!"<<len <<"!=" <<data_long<<endl;
      return -1;}
  ptrSit = DataBuf;
  memcpy((char*)(&pb_trajectory), ptrSit, data_long);


  uint64 Timestamp_ms = pb_trajectory.pb_header.camera_timestamp;
  struct timeval tStart;
  tStart.tv_sec = Timestamp_ms/1000 ;
  struct tm *info;
  info = gmtime(&tStart.tv_sec);
  printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
  printf("%d:%d:%d \n", info->tm_hour+8,info->tm_min, info->tm_sec);

  cout<<"trajectory_points_num = "<<pb_trajectory.trajectory_points_num<<endl;

  data_long = sizeof(TrajectoryPoint);
  pb_trajectory.trajectory_points.clear();
  for(int i = 0; i < pb_trajectory.trajectory_points_num ; i++)
    {
      TrajectoryPoint trajectory_point_;
      len = fread(DataBuf, sizeof(char), data_long, PbResult);
      if (len != data_long)
        {
          cout<<"read Traffic data long Error 0 !!!"<<endl;
          return -1;
        }
      ptrSit = DataBuf;
      memcpy((char*)(&trajectory_point_), ptrSit, data_long);

      pb_trajectory.trajectory_points.push_back(trajectory_point_);
    }
  read_times_++;
  cout<<"Read PbTrajectory subsquence: "<<read_times_<<endl<<endl;
  return 1;

}


int ReadLocationData( FILE* LoResult,
                      localization::LocalizationEstimate &localization_,
                      int &read_times_){
  int len = 0;
  char DataBuf[65535] = { 0 };
  char* ptrSit = DataBuf;
  int data_long = sizeof(localization_);
  uint64 Timestamp_ms;

  len = fread(DataBuf, sizeof(char), data_long, LoResult);
  if (len != data_long){
      cout<<"Read Location data Finish !!!"<<len <<"!=" <<data_long<<endl;
      return -1;}

    ptrSit = DataBuf;
    memcpy((char*)(&localization_), ptrSit, sizeof(unsigned long long));
    Timestamp_ms = localization_.timestamp_ms;
    struct timeval tStart;
    tStart.tv_sec = Timestamp_ms/1000 ;
    struct tm *info;
    info = gmtime(&tStart.tv_sec);
    printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
    printf("%d:%d:%d \n", info->tm_hour+8,info->tm_min, info->tm_sec);
    read_times_++;
    cout<<"Read Location subsquence: "<<read_times_<<endl;
  return 1;


}

int ReadTrafficData( FILE* TrResult,
                     perception::TrafficLightDetection  &traffic_detection_,
                     int &read_times_){
  int len = 0;
  char DataBuf[65535] = { 0 };
  char* ptrSit = DataBuf;
  int data_long = 8*2;
  uint64 Timestamp_ms;

  len = fread(DataBuf, sizeof(char), data_long, TrResult);
  if (len != data_long){
      cout<<"Read Traffic Finish !!!"<<len <<"!=" <<data_long<<endl;
      return -1;}
  ptrSit = DataBuf;
  memcpy((char*)(&traffic_detection_), ptrSit, data_long);

  Timestamp_ms = traffic_detection_.camera_timestamp;
  struct timeval tStart;
  tStart.tv_sec = Timestamp_ms/1000 ;
  struct tm *info;
  info = gmtime(&tStart.tv_sec);
  printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
  printf("%d:%d:%d \n", info->tm_hour+8,info->tm_min, info->tm_sec);
  cout<<"traffic_detection_.frame_num = "<<traffic_detection_.frame_num<<endl;

  perception::TrafficLight traffic_light_;
  data_long = sizeof(traffic_light_);
  traffic_detection_.traffic_light.clear();
  for(int i = 0; i < traffic_detection_.frame_num ; i++)
    {
      len = fread(DataBuf, sizeof(char), data_long, TrResult);
      if (len != data_long)
        {
          cout<<"read Traffic data long Error !!!"<<endl;
          return -1;
        }
      ptrSit = DataBuf;
      memcpy((char*)(&traffic_light_), ptrSit, data_long);
      traffic_detection_.traffic_light.push_back(traffic_light_);
    }

  read_times_++;
  cout<<"Read Traffic subsquence: "<<read_times_<<endl<<endl;
  return 1;

}


int ReadPeridictionData( FILE*  PeResult,
                         prediction::PredictionObstacles &obstacles_ ,
                         int &read_times_){
  int len = 0;
  char DataBuf[65535] = { 0 };
  char* ptrSit = DataBuf;

  //int data_long = sizeof(chassis_);
  //uint64 Timestamp_ms;

  int data_long = sizeof(obstacles_.header);
  //cout<<"obstacles_num_: "<<obstacles_num_<<endl;

  len = fread(DataBuf, sizeof(char), data_long, PeResult);
  if (len != data_long){
      cout<<"Read Prediction Data Finish ... "<<len <<"!=" <<data_long<<endl<<endl;
      return -1;
  }
  ptrSit = DataBuf;
  memcpy((char*)(&obstacles_.header), ptrSit, data_long);


  struct timeval tStart;
  tStart.tv_sec = obstacles_.header.radar_timestamp/1000 ;
  struct tm *info;
  info = gmtime(&tStart.tv_sec);
  printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
  printf("%d:%d:%d ", info->tm_hour+8,info->tm_min, info->tm_sec);

  data_long = 8;
  len = fread(DataBuf, sizeof(char), data_long, PeResult);
  if (len != data_long){
      cout<<"ERROR Prediction 2 ... "<<len <<"!=" <<data_long<<endl;
      return -1;
  }
  ptrSit = DataBuf;
  memcpy((char*)(&obstacles_.prediction_obstacle_num), ptrSit, data_long);

  cout<<"obstacles_.prediction_obstacle_num = "
      <<obstacles_.prediction_obstacle_num <<endl;
  //if(obstacles_.prediction_obstacle_num != 2) return -1;

  obstacles_.prediction_obstacle.clear();

  for( int u =0 ; u < obstacles_.prediction_obstacle_num ; u++){

     prediction::PredictionObstacle prediction_obstacle_;

      data_long = 96;//8 + 8*3 + 8 +8*3 + 8*3 + 8 ;
      len = fread(DataBuf, sizeof(char), data_long, PeResult);
      if (len != data_long){
          cout<<"ERROR Prediction 3 ... "<<len <<"!=" <<data_long<<endl;
          return -1;
      }
      ptrSit = DataBuf;
      memcpy((char*)(&prediction_obstacle_.perception_obstacle.id), ptrSit, data_long);
#if 0
      cout<<"prediction_obstacle_.perception_obstacle.id = "<<
             prediction_obstacle_.perception_obstacle.id<<endl;
      cout<<prediction_obstacle_.perception_obstacle.position.x <<" = " << 1111<<endl;
      cout<<prediction_obstacle_.perception_obstacle.position.y <<" = " << 1112<<endl;
      cout<<prediction_obstacle_.perception_obstacle.position.z <<" = " << 1113<<endl;
      cout<<prediction_obstacle_.perception_obstacle.theta      <<" = " << 1114<<endl;
      cout<<prediction_obstacle_.perception_obstacle.velocity.x <<" = " << 1115<<endl;
      cout<<prediction_obstacle_.perception_obstacle.velocity.y <<" = " << 1116<<endl;
      cout<<prediction_obstacle_.perception_obstacle.velocity.z <<" = " << 1117<<endl;
      cout<<prediction_obstacle_.perception_obstacle.length     <<" = " << 1118<<endl;
      cout<<prediction_obstacle_.perception_obstacle.width      <<" = " << 1119<<endl;
      cout<<prediction_obstacle_.perception_obstacle.height     <<" = " << 1120<<endl;
#endif
      cout<<"prediction_obstacle_.perception_obstacle.polygon_num = "<<
             prediction_obstacle_.perception_obstacle.polygon_num<<endl;

      //if(prediction_obstacle_.perception_obstacle.polygon_num != 6) return -1;
      perception::Point point_ ;
      prediction_obstacle_.perception_obstacle.polygon_point.clear();
      for(int v = 0; v < prediction_obstacle_.perception_obstacle.polygon_num ; v++ )
        {

          data_long = 8*3 ;
          len = fread(DataBuf, sizeof(char), data_long, PeResult);
          if (len != data_long){
              cout<<"ERROR Prediction 4 ... "<<len <<"!=" <<data_long<<endl;
              return -1;
          }
          ptrSit = DataBuf;
          memcpy((char*)(&point_.x), ptrSit, data_long);
//            cout<<"v= "<< v <<endl;
//            cout<<point_.x <<" = " <<3333<<endl;
//            cout<<point_.y <<" = " <<3334<<endl;
//            cout<<point_.z <<" = " <<3335<<endl;
//            cout<<""<<endl;

          prediction_obstacle_.perception_obstacle.polygon_point.push_back(point_);
        }
      //*************read data strat ***************************************************//
      data_long = 8*2 + 8 ; //gggg
      memset(DataBuf,0,1000);
      len = fread(DataBuf, sizeof(char), data_long, PeResult);
      if (len != data_long){
          cout<<"ERROR Prediction 5 ... "<<len <<"!=" <<data_long<<endl;
          return -1;
      }
      ptrSit = DataBuf;
      memcpy((char*)(&prediction_obstacle_.perception_obstacle.tracking_time), ptrSit, data_long);
      //*************read data end *****************************************************//
//     cout<<prediction_obstacle_.perception_obstacle.tracking_time <<" = " << 30000<<endl;
//     cout<<prediction_obstacle_.perception_obstacle.type <<" = " << perception::VEHICLE<<endl;
//     cout<<prediction_obstacle_.perception_obstacle.confidence <<" = " << 30001<<endl<<endl;

      data_long = 8*3 ;//gggg
      memset(DataBuf,0,1000);
      len = fread(DataBuf, sizeof(char), data_long, PeResult);
      if (len != data_long){
          cout<<"ERROR Prediction 6 ... "<<len <<"!=" <<data_long<<endl;
          return -1;
      }
      ptrSit = DataBuf;
      memcpy((char*)(&prediction_obstacle_.timestamp), ptrSit, data_long);

//    cout<<prediction_obstacle_.timestamp <<"!="<< 111111<<endl;
//    cout<<prediction_obstacle_.predicted_period <<"!="<< 888888<<endl;
      cout<<"prediction_obstacle_.trajectory_num = "
          <<prediction_obstacle_.trajectory_num<<endl;
//    if(prediction_obstacle_.trajectory_num != 2){return -1;}

      prediction_obstacle_.trajectory.clear();
      for(int w = 0 ; w < prediction_obstacle_.trajectory_num ; w++ )
        {
          prediction::Trajectory trajectory_;
          data_long = 8 + 8 ;//gggg
          len = fread(DataBuf, sizeof(char), data_long, PeResult);
          if (len != data_long)
            {
             cout<<"ERROR Prediction 7 ... "<<len <<"!=" <<data_long<<endl;
             return -1;
            }
          ptrSit = DataBuf;
          memcpy((char*)(&trajectory_.probability), ptrSit, data_long);
          cout<<"trajectory_.trajectory_point_num = "
              << trajectory_.trajectory_point_num <<endl;

          trajectory_.trajectory_point.clear();
          for(int x = 0; x < trajectory_.trajectory_point_num ; x++ )
           {
             TrajectoryPoint trajectory_point_;
             data_long = sizeof(trajectory_point_);
             len = fread(DataBuf, sizeof(char), data_long, PeResult);
             if (len != data_long){
                 cout<<"ERROR Prediction 8 ... "<<len <<"!=" <<data_long<<endl;
                 return -1;
             }
             ptrSit = DataBuf;
             memcpy((char*)(&trajectory_point_), ptrSit, data_long);
             trajectory_.trajectory_point.push_back(trajectory_point_);
//               cout<<"x = "<<x<<endl;
//               cout<<"trajectory_point_.path_point.x = "
//                   <<trajectory_point_.path_point.x<<endl;
//               cout<<"trajectory_point_.v = "
//                   <<trajectory_point_.v<<endl;
           }
         prediction_obstacle_.trajectory.push_back(trajectory_);

        }
      obstacles_.prediction_obstacle.push_back(prediction_obstacle_);
    }

  data_long = 8*2;//gggg
  len = fread(DataBuf, sizeof(char), data_long, PeResult);
  if (len != data_long){
      cout<<"ERROR Prediction 9 ... "<<len <<"!=" <<data_long<<endl;
      return -1;
  }
  ptrSit = DataBuf;
  memcpy((char*)(&obstacles_.end_timestamp), ptrSit, data_long);

  tStart.tv_sec = obstacles_.end_timestamp/1000 ;
  info = gmtime(&tStart.tv_sec);
  printf("start %d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
  printf("%d:%d:%d \n ", info->tm_hour+8,info->tm_min, info->tm_sec);

  read_times_++;
  cout<<"Read Prediction subsquence: "<<read_times_<<endl<<endl;
  //cout<<"Read Prediction Data complete !!!"<<endl;

  return 1;

}

int ReadChassisData(FILE* ChResult, canbus::Chassis &chassis_,int &read_times_){

  int len = 0;
  char DataBuf[65535] = { 0 };
  char* ptrSit = DataBuf;

  int data_long = sizeof(chassis_);
  uint64 Timestamp_ms;


  len = fread(DataBuf, sizeof(char), data_long, ChResult);
  if (len != data_long)
    {
      cout<<"Read Chassisa data Finish !!!"<<len <<"!=" <<data_long<<endl;
      return -1;
    }

    ptrSit = DataBuf;
    memcpy((char*)(&chassis_), ptrSit, sizeof(unsigned long long));
    Timestamp_ms = chassis_.timestamp_ms;
    struct timeval tStart;
    tStart.tv_sec = Timestamp_ms/1000 ;
    struct tm *info;
    info = gmtime(&tStart.tv_sec);
    printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
    printf("%d:%d:%d \n", info->tm_hour+8,info->tm_min, info->tm_sec);
    read_times_++;
    cout<<"Read Chassis subsquence: "<<read_times_<<endl<<endl;
  return 1;

}


int MakeChassisData(FILE*  ChResult,
                    uint64 Timestamp_ms,
                    int    FrameNum ){

  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;
  int data_long = sizeof(canbus::Chassis);
  canbus::Chassis make_chassis_data;
  int i = FrameNum;
//*******************************chassis data write start ***************************//

      make_chassis_data.timestamp_ms = Timestamp_ms;
      make_chassis_data.speed_mps = i * 0.1;
      make_chassis_data.engine_started = true;
      make_chassis_data.engine_rpm = i * 0.5;
      make_chassis_data.odometer_m = 50000 + i * 10;

      make_chassis_data.battery_range_km = 100 - i*0.6;
      make_chassis_data.throttle_percentage = 0.95;
      make_chassis_data.brake_percentage = 0.8;
      make_chassis_data.steering_percentage = 0.6;
      make_chassis_data.steering_torque_nm = 1200;

      make_chassis_data.parking_brake = true;
      make_chassis_data.wiper = false;
      make_chassis_data.driving_mode = COMPLETE_AUTO_DRIVE;
      make_chassis_data.gear_location = GEAR_DRIVE;

      make_chassis_data.signal.emergency_light = false;
      make_chassis_data.signal.high_beam = false;
      make_chassis_data.signal.horn = true;
      make_chassis_data.signal.low_beam = true;
      make_chassis_data.signal.turn_signal = TURN_LEFT;

      make_chassis_data.has_data = true;
      memcpy(ChBuf, &make_chassis_data, data_long);
      ptr_data = ChBuf;
      real_long  = fwrite(ptr_data, sizeof(char), data_long, ChResult);
      //(prt,one word,data long,file)
      if(real_long != data_long){
          std::cout<<"Error write data real_long != data_long"<<std::endl;
          return -1;
        }
   cout<<" "<<endl;

}

int MakeLocationData(FILE* LoResult,uint64 Timestamp_ms,int FrameNum)
{
  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int data_long ;

  localization::LocalizationEstimate localization_;
  //cout<<"location data write start ************************!!!"<<endl;
  data_long = sizeof(localization::LocalizationEstimate);

     localization_.timestamp_ms = Timestamp_ms;
     localization_.pose.position.x = 0.2;
     localization_.pose.position.y = 0.3;
     localization_.pose.position.z = 0.0;

//     localization_.pose.orientation.qw = 0.3;
//     localization_.pose.orientation.qx = 0.1;
//     localization_.pose.orientation.qy = 0.5;
//     localization_.pose.orientation.qz = 0.2;

     localization_.pose.linear_velocity.x =  0.2;
     localization_.pose.linear_velocity.y =  0.3;
     localization_.pose.linear_velocity.z = 0.0;

//     localization_.pose.linear_acceleration.x = 0.01;
//     localization_.pose.linear_acceleration.y = 0.05;
//     localization_.pose.linear_acceleration.z = 0.0;

//     localization_.pose.angular_velocity.x = 0.18;
//     localization_.pose.angular_velocity.y = 0.28;
//     localization_.pose.angular_velocity.z = 0.38;

     //localization_.pose.heading = 0.01 ;

     localization_.pose.linear_acceleration_vrf.x = 0.12 ;
     localization_.pose.linear_acceleration_vrf.y = 0.01 ;
     localization_.pose.linear_acceleration_vrf.z = 0.02 ;

     localization_.pose.angular_velocity_vrf.x = 0.1;
     localization_.pose.angular_velocity_vrf.y = 0.2;
     localization_.pose.angular_velocity_vrf.z = 0.3;

     localization_.pose.euler_angles.x = 0.0015 ;
     localization_.pose.euler_angles.y = 0.0005 ;
     localization_.pose.euler_angles.z = 0.0003 ;

     localization_.pose.point_llh.lon = 0.02;
     localization_.pose.point_llh.lat = 0.02;
     localization_.pose.point_llh.height = 1;

     localization_.pose.has_data = true;

     memcpy(ChBuf, &localization_.timestamp_ms, data_long);
     ptr_data = ChBuf;
     int real_long  =
     fwrite(ptr_data, sizeof(char), data_long, LoResult);

     if(real_long != data_long){
          std::cout<<"Error location write!!! "<< real_long <<" != "
                   <<data_long<<std::endl;
          return -1;
        }
  cout<<"Location Data FrameNum = "<<FrameNum<<endl;
}


int MakeTrafficData(FILE* TrResult,
                    uint64 Timestamp_ms,
                    int32  TrafficFram,
                    int    FrameNum //subsequence
                    ){

  perception::TrafficLightDetection  traffic_detection_;
  int frame_num_ = TrafficFram;
  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;
  int header_long;

  traffic_detection_.camera_timestamp = Timestamp_ms;
  traffic_detection_.frame_num = frame_num_;
  header_long = 8*2;
  memcpy(ChBuf, &traffic_detection_, header_long);
  ptr_data = ChBuf;
  real_long  =
  fwrite(ptr_data, sizeof(char), header_long, TrResult);
  if(real_long != header_long){
      std::cout<<"Error traffic_light 0!!! "<< real_long
               <<" != "<< header_long   <<std::endl;
      return -1;
    }


  perception::TrafficLight traffic_light_;
  for(int r = 0; r < frame_num_ ; r++)
    {
      traffic_light_.color = perception::GREEN;
      traffic_light_.id = 1;
      traffic_light_.confidence = 0.99;
      traffic_light_.tracking_time = Timestamp_ms + 6;
      //***********write data*********************************************//
      header_long =  sizeof(traffic_light_);
      memcpy(ChBuf, &traffic_light_, header_long);
      char* ptr_data = ChBuf;
      int real_long  =
      fwrite(ptr_data, sizeof(char), header_long, TrResult);
      if(real_long != header_long){
          std::cout<<"Error traffic_light 1!!! "<< real_long
                   <<" != "<< header_long   <<std::endl;
          return -1;
        }
      //***********write data********************************************//
      traffic_detection_.traffic_light.push_back(traffic_light_);
    }
   cout<<"Traffic Data FrameNum = "<< FrameNum <<endl;
}


int MakePeridictionData(FILE*  PeResult,
                        uint64 Timestamp_ms,
                        int32  ObstacleNum,
                        int32  PolygonNum,
                        int32  TrajectoryNum,
                        int32  TrajectoryPointNum,
                        int32  FrameNum //subsequence
                        ){
  //int prediction_data_frame = 64;
  auto time_ms = Timestamp_ms;
  int32 obstacles_num = ObstacleNum;
  int32 trajectory_num = TrajectoryNum;
  int32 trajectory_point_num = TrajectoryPointNum;
  int32 polygon_num = PolygonNum; //多边形边数

  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;

    prediction::PredictionObstacles obstacles_;
    int header_long = sizeof(obstacles_.header);
    obstacles_.header.camera_timestamp = time_ms +15;
    obstacles_.header.lidar_timestamp = time_ms +5;
    obstacles_.header.radar_timestamp = time_ms +10;
    obstacles_.start_timestamp = time_ms;
    obstacles_.header.sequence_num = FrameNum;
    obstacles_.prediction_obstacle_num = obstacles_num;

    header_long = header_long + 8;
    memcpy(ChBuf, &obstacles_.header, header_long);
    ptr_data = ChBuf;
    real_long  =
    fwrite(ptr_data, sizeof(char), header_long, PeResult);
    if(real_long != header_long){
        std::cout<<"Error 1!!! "<< real_long <<" != "<< header_long <<std::endl;
        return -1;
      }

    prediction::PredictionObstacle  Obstacle_;
    for(int k = 0; k < obstacles_num ; k++)
    {
       Obstacle_.perception_obstacle.id = 1110 ;
       Obstacle_.perception_obstacle.position.x = 1111;
       Obstacle_.perception_obstacle.position.y = 1112;
       Obstacle_.perception_obstacle.position.z = 1113;
       Obstacle_.perception_obstacle.theta = 1114;
       Obstacle_.perception_obstacle.velocity.x = 1115;
       Obstacle_.perception_obstacle.velocity.y = 1116;
       Obstacle_.perception_obstacle.velocity.z = 1117;
       Obstacle_.perception_obstacle.length = 1118 ;
       Obstacle_.perception_obstacle.width =  1119;
       Obstacle_.perception_obstacle.height = 1120;
       Obstacle_.perception_obstacle.polygon_num = polygon_num;

       header_long = 8*12;
       memcpy(ChBuf, &Obstacle_.perception_obstacle.id, header_long);
       char* ptr_data = ChBuf;
       int real_long  =
       fwrite(ptr_data, sizeof(char), header_long, PeResult);
       if(real_long != header_long){
           std::cout<<"Error 1!!! "<< real_long <<" != "<< header_long <<std::endl;
           return -1;
         }

       perception::Point point_;
       for(int l=0;l<polygon_num;l++){
           point_.x =3333;
           point_.y =3334;
           point_.z =3335;
           //***********write data*********************************************//
           header_long =  8*3;
           memcpy(ChBuf, &point_.x, header_long);
           char* ptr_data = ChBuf;
           int real_long  =
           fwrite(ptr_data, sizeof(char), header_long, PeResult);
           if(real_long != header_long){
               std::cout<<"Error 2!!! "<< real_long <<" != "<< header_long <<std::endl;
               return -1;
             }
           //***********write data********************************************//
           Obstacle_.perception_obstacle.polygon_point.push_back(point_);
         }

        Obstacle_.perception_obstacle.tracking_time = 30000;
        Obstacle_.perception_obstacle.type = perception::VEHICLE;
        Obstacle_.perception_obstacle.confidence = 30001;
        //***********write data*********************************************//
        header_long =  8*2 +8; //gggg
        memcpy(ChBuf, &Obstacle_.perception_obstacle.tracking_time, header_long);
        ptr_data = ChBuf;
        real_long  =
        fwrite(ptr_data, sizeof(char), header_long, PeResult);
        if(real_long != header_long){
            std::cout<<"Error 3!!! "<< real_long <<" != "<< header_long <<std::endl;
            return -1;
          }
        //***********write data********************************************//
       Obstacle_.timestamp = 111111;
       Obstacle_.predicted_period = 888888;
       Obstacle_.trajectory_num = trajectory_num;
       //uint64 timestamp = 11111111;
       //***********write data*********************************************//
       header_long =  8*3;//gggg
       memcpy(ChBuf, &Obstacle_.timestamp, header_long);
       ptr_data = ChBuf;
       real_long  =
       fwrite(ptr_data, sizeof(char), header_long, PeResult);
       if(real_long != header_long){
           std::cout<<"Error 4!!! "<< real_long <<" != "<< header_long <<std::endl;
           return -1;
         }
       //***********write data********************************************//
       prediction::Trajectory trajectory_;
       for(int m = 0; m<trajectory_num;m++){

           trajectory_.probability = 1;
           trajectory_.trajectory_point_num = trajectory_point_num;
           //***********write data*********************************************//
           header_long =  8 +8; //gggg
           memcpy(ChBuf, &trajectory_.probability, header_long);
           char* ptr_data = ChBuf;
           int real_long  =
           fwrite(ptr_data, sizeof(char), header_long, PeResult);
           if(real_long != header_long){
               std::cout<<"Error 5!!! "<< real_long <<" != "<< header_long <<std::endl;
               return -1;
             }
           //***********write data********************************************//
           TrajectoryPoint trajectory_point_;
           for(int n = 0;n<trajectory_point_num;n++){

               trajectory_point_.path_point.x = 2;
               trajectory_point_.path_point.y = 2;
               trajectory_point_.path_point.z = 2;
               trajectory_point_.path_point.s = 2;
               trajectory_point_.path_point.theta = 2;
               trajectory_point_.path_point.kappa = 2;
               trajectory_point_.path_point.dkappa = 2;
               trajectory_point_.path_point.ddkappa = 2;
               trajectory_point_.a = 2;
               trajectory_point_.v = 5;
               trajectory_point_.relative_time = 2;
               //***********write data*********************************************//
               header_long =  sizeof(TrajectoryPoint);
               memcpy(ChBuf, &trajectory_point_, header_long);
               char* ptr_data = ChBuf;
               int real_long  =
               fwrite(ptr_data, sizeof(char), header_long, PeResult);
               if(real_long != header_long){
                   std::cout<<"Error 6!!! "<< real_long <<" != "<< header_long <<std::endl;
                   return -1;
                 }
               //***********write data********************************************//
               trajectory_.trajectory_point.push_back(trajectory_point_);

             }
           Obstacle_.trajectory.push_back(trajectory_);
         }

       }
       obstacles_.start_timestamp = time_ms;
       obstacles_.end_timestamp = Clock::NowInMs();
       //***********write data*********************************************//
       header_long =  8*2;
       memcpy(ChBuf, &obstacles_.start_timestamp, header_long);
       ptr_data = ChBuf;
       real_long  =
       fwrite(ptr_data, sizeof(char), header_long, PeResult);
       if(real_long != header_long){
           std::cout<<"Error 7!!! "<< real_long <<" != "<< header_long <<std::endl;
           return -1;
         }
       //***********write data********************************************//
       obstacles_.prediction_obstacle.push_back(Obstacle_);
       cout<<"FrameNum = "<<FrameNum<<endl;

}

int MakeTrajectoryData(FILE* PbResult,PbHeader Timestamp){

  PbTrajectory  pb_trajectory;
  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;
  int header_long;
  cout<<"Timestamp = "<<Timestamp.camera_timestamp<<endl;
  header_long = sizeof(PbHeader);
  memcpy(ChBuf, &Timestamp, header_long);
  ptr_data = ChBuf;
  real_long  =
  fwrite(ptr_data, sizeof(char), header_long, PbResult);
  if(real_long != header_long){
      std::cout<<"Error PbTrajectory make data 0!!! "<< real_long
               <<" != "<< header_long   <<std::endl;
      return -1;
    }
  uint64 Timestamp_ms = Timestamp.camera_timestamp;
  struct timeval tStart;
  tStart.tv_sec = Timestamp_ms/1000 ;
  struct tm *info;
  info = gmtime(&tStart.tv_sec);
  printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
  printf("%d:%d:%d \n", info->tm_hour+8,info->tm_min, info->tm_sec);


  header_long = sizeof(VehiclePositonState) + 8  ;
  memcpy(ChBuf, &pb_trajectory.vechicle_state, header_long);
  ptr_data = ChBuf;
  real_long  =
  fwrite(ptr_data, sizeof(char), header_long, PbResult);
  if(real_long != header_long){
      std::cout<<"Error PbTrajectory make data 1!!! "<< real_long
               <<" != "<< header_long   <<std::endl;
      return -1;
    }

  cout<<"trajectory_points_num = "<<pb_trajectory.trajectory_points_num<<endl;

  TrajectoryPoint trajectory_point_;
   header_long = sizeof(TrajectoryPoint) ;
  for(int r = 0; r < pb_trajectory.trajectory_points_num ; r++)
    {    
      memcpy(ChBuf, &trajectory_point_, header_long);
      ptr_data = ChBuf;
      real_long  =
      fwrite(ptr_data, sizeof(char), header_long, PbResult);
      if(real_long != header_long){
          std::cout<<"Error PbTrajectory make data 1!!! "<< real_long
                   <<" != "<< header_long   <<std::endl;
          return -1;
        }
      //cout<<"point_num = "<<r<<endl;
    }
   //cout<<"Trajectory Data FrameNum = "<< FrameNum++<<endl;
}
///******************************************************************//

int WriteLocationData(FILE* LoResult,
                     localization::LocalizationEstimate localization_ ){

    char ChBuf[65535] = { 0 };
    char* ptr_data = ChBuf;
    int data_long ;

    //cout<<"location data write start ************************!!!"<<endl;
    data_long = sizeof(localization::LocalizationEstimate);
    memcpy(ChBuf, &localization_.timestamp_ms, data_long);
    ptr_data = ChBuf;
    int real_long  =
    fwrite(ptr_data, sizeof(char), data_long, LoResult);
    if(real_long != data_long){
       std::cout<<"Error location write!!! "<< real_long <<" != "
                <<data_long<<std::endl;
       return -1;
     }

}

int WriteTrajectoryData(FILE* PbResult,PbTrajectory  pb_trajectory){

  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;
  int header_long;
  cout<<"Timestamp = "<<pb_trajectory.pb_header.camera_timestamp<<endl;
  header_long = sizeof(PbHeader);
  memcpy(ChBuf, &pb_trajectory, header_long);
  ptr_data = ChBuf;
  real_long  =
  fwrite(ptr_data, sizeof(char), header_long, PbResult);
  if(real_long != header_long){
      std::cout<<"Error PbTrajectory make data 0!!! "<< real_long
               <<" != "<< header_long   <<std::endl;
      return -1;
    }
  
  header_long = sizeof(VehiclePositonState) + 8  ;
  memcpy(ChBuf, &pb_trajectory.vechicle_state, header_long);
  ptr_data = ChBuf;
  real_long  =
  fwrite(ptr_data, sizeof(char), header_long, PbResult);
  if(real_long != header_long){
      std::cout<<"Error PbTrajectory make data 1!!! "<< real_long
               <<" != "<< header_long   <<std::endl;
      return -1;
    }

  cout<<"trajectory_points_num = "<<pb_trajectory.trajectory_points_num<<endl;


  header_long = sizeof(TrajectoryPoint) ;
  for(int r = 0; r < pb_trajectory.trajectory_points_num ; r++)
    {
      memcpy(ChBuf, &pb_trajectory.trajectory_points.at(r), header_long);
      ptr_data = ChBuf;
      real_long  =
      fwrite(ptr_data, sizeof(char), header_long, PbResult);
      if(real_long != header_long){
          std::cout<<"Error PbTrajectory make data 1!!! "<< real_long
                   <<" != "<< header_long   <<std::endl;
          return -1;
        }
    }
}

int WriteChassisData( FILE* ChResult,canbus::Chassis chassis_ ){
    
  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;
  int data_long = sizeof(canbus::Chassis);

//*******************************chassis data write start ***************************//
      memcpy(ChBuf, &chassis_, data_long);
      ptr_data = ChBuf;
      real_long  = fwrite(ptr_data, sizeof(char), data_long, ChResult);
      //(prt,one word,data long,file)
      if(real_long != data_long){
          std::cout<<"Error write data real_long != data_long"<<std::endl;
          return -1;
        }
}


int WriteTrafficData( FILE* TrResult,
                      perception::TrafficLightDetection  traffic_detection_){

  int frame_num_ = traffic_detection_.frame_num;
  char ChBuf[65535] = { 0 };
  char* ptr_data = ChBuf;
  int real_long ;
  int header_long;

  header_long = 8*2;
  memcpy(ChBuf, &traffic_detection_, header_long);
  ptr_data = ChBuf;
  real_long  =
  fwrite(ptr_data, sizeof(char), header_long, TrResult);
  if(real_long != header_long){
      std::cout<<"Error traffic_light 0!!! "<< real_long
               <<" != "<< header_long   <<std::endl;
      return -1;
    }

  for(int r = 0; r < frame_num_ ; r++)
    {
      //***********write data*********************************************//
      header_long =  sizeof(perception::TrafficLight);
      memcpy(ChBuf, &traffic_detection_.traffic_light.at(r), header_long);
      char* ptr_data = ChBuf;
      int real_long  =
      fwrite(ptr_data, sizeof(char), header_long, TrResult);
      if(real_long != header_long){
          std::cout<<"Error traffic_light 1!!! "<< real_long
                   <<" != "<< header_long   <<std::endl;
          return -1;
        }
      //***********write data********************************************//
    }

}

int WritePeridictionData( FILE*  PeResult,
                          prediction::PredictionObstacles obstacles_ ){

    char ChBuf[65535] = { 0 };
    char* ptr_data = ChBuf;
    int real_long ;

    int header_long = sizeof(obstacles_.header) + 8;
    memcpy(ChBuf, &obstacles_.header, header_long);
    ptr_data = ChBuf;
    real_long  =
    fwrite(ptr_data, sizeof(char), header_long, PeResult);
    if(real_long != header_long){
        std::cout<<"Error 1!!! "<< real_long <<" != "<< header_long <<std::endl;
        return -1;
      }

    for(int k = 0; k < obstacles_.prediction_obstacle_num ; k++)
    {
     header_long = 8*12;
     memcpy(ChBuf, &obstacles_.prediction_obstacle.at(k).perception_obstacle.id, header_long);
     char* ptr_data = ChBuf;
     int real_long  =
     fwrite(ptr_data, sizeof(char), header_long, PeResult);
     if(real_long != header_long){
           std::cout<<"Error 1!!! "<< real_long <<" != "<< header_long <<std::endl;
           return -1;
         }

     for(int l=0;l<obstacles_.prediction_obstacle.at(k).perception_obstacle.polygon_num;l++)
      {
        //***********write data*********************************************//
        header_long =  8*3;
        memcpy(ChBuf,
        &obstacles_.prediction_obstacle.at(k).perception_obstacle.polygon_point.at(l),
        header_long);
        char* ptr_data = ChBuf;
        int real_long  =
        fwrite(ptr_data, sizeof(char), header_long, PeResult);
        if(real_long != header_long){
               std::cout<<"Error 2!!! "<< real_long <<" != "<< header_long <<std::endl;
               return -1;
          }
        //***********write data********************************************//
       }
      //***********write data*********************************************//
        header_long =  8*2 +8; //gggg
        memcpy(ChBuf,
               &obstacles_.prediction_obstacle.at(k).perception_obstacle.tracking_time,
               header_long);
        ptr_data = ChBuf;
        real_long  =
        fwrite(ptr_data, sizeof(char), header_long, PeResult);
        if(real_long != header_long){
            std::cout<<"Error 3!!! "<< real_long <<" != "<< header_long <<std::endl;
            return -1;
          }
        //***********write data********************************************//


       //***********write data*********************************************//
       header_long =  8*3;//gggg
       memcpy(ChBuf, &obstacles_.prediction_obstacle.at(k).timestamp, header_long);
       ptr_data = ChBuf;
       real_long  =
       fwrite(ptr_data, sizeof(char), header_long, PeResult);
       if(real_long != header_long){
           std::cout<<"Error 4!!! "<< real_long <<" != "<< header_long <<std::endl;
           return -1;
         }
       //***********write data********************************************//

       for(int m = 0; m<obstacles_.prediction_obstacle.at(k).trajectory_num;m++){

         //***********write data*********************************************//
         header_long =  8 +8; //gggg
         memcpy(ChBuf,
                &obstacles_.prediction_obstacle.at(k).trajectory.at(m).probability,
                header_long);
         char* ptr_data = ChBuf;
         int real_long  =
         fwrite(ptr_data, sizeof(char), header_long, PeResult);
         if(real_long != header_long){
             std::cout<<"Error 5!!! "<< real_long <<" != "<< header_long <<std::endl;
             return -1;
           }
         //***********write data********************************************//
         for(int n = 0;
             n < obstacles_.prediction_obstacle.at(k).trajectory.at(m).trajectory_point_num;
             n++)
           {
            //***********write data*********************************************//
            header_long =  sizeof(TrajectoryPoint);
            memcpy(ChBuf,
            &obstacles_.prediction_obstacle.at(k).trajectory.at(m).trajectory_point.at(n).path_point,
            header_long);
            char* ptr_data = ChBuf;
            int real_long  =
            fwrite(ptr_data, sizeof(char), header_long, PeResult);
            if(real_long != header_long){
                std::cout<<"Error 6!!! "<< real_long <<" != "<< header_long <<std::endl;
                return -1;
              }
            //***********write data********************************************//
           }
       }

    }
   //***********write data*********************************************//
   header_long =  8*2;
   memcpy(ChBuf, &obstacles_.start_timestamp, header_long);
   ptr_data = ChBuf;
   real_long  =
   fwrite(ptr_data, sizeof(char), header_long, PeResult);
   if(real_long != header_long){
       std::cout<<"Error 7!!! "<< real_long <<" != "<< header_long <<std::endl;
       return -1;
     }
   //***********write data********************************************//
}




#endif
} //namespace planning

