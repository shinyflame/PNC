#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "common/struct.h"
#include "math/util.h"
#include "iostream"
#include "lattice/behavior/feasible_region.h"
#include "lattice/trajectory_generation/end_condition_sampler.h"
#include "json/json.h"
#include "common/util/factory.h"
#include "common/get_now_time.h"
#include "common/g_test.h"
#include "common/speed/st_boundary.h"
#include "math/cartesian_frenet_conversion.h"
#include "math/euler_angles_zxy.h"
#include "math/quaternion.h"
#include "math/search.h"
#include "math/aabox2d.h"
#include "math/aaboxkdtree2d.h"
#include "math/box2d.h"
#include "math/angle.h"
#include "common/trajectory/discretized_trajectory.h"
#include "common/trajectory/publishable_trajectory.h"
#include "common/ego_info.h"
#include "common/frame.h"
#include "common/write_read_file.h"
#include "common/coordinate_convert.h"
#include "reference_line/reference_line_provider.h"
#include "planner/lattice/lattice_planner.h"
#include "std_planning.h"
#include "math/box2d.h"
#include "../map/common/serialize_data.h"

#include <QVector>
#include <QTime>
#include <QTimer>

using namespace planning ;
using hdmap::PncInformation;

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;


/////////////////////////////////////////Color Explanation start////////////////////////

//  1.GREEN:   OTHER MAPS  CENTER POINTS;
//  2.YELLOW:CURRENT MAP    CENTER POINTS;
//  3.LIGTH GRAY : CURRENT ROUTE LANE BOUNDARY;
//  4.BLUE: CURRENT ROUTE CENTER POINTS;
//  5.DARKGRAY: CURRENT REFERENCELINE ANCHOR POINTS;
//  6.BLACK:CURRENT REFERENCE POINTS
//  7.RED: CURRENT TRAJECTORY POINTS

//////////////////////////////////////////Color Explanation end///////////////////



/////////////////////////////////config, map, recode path////////////////////////
string  /// 1. Qingfeng Park
        /*hdmap_path1 = "/home/shiny/"
                          "/map/map_bin_files/qingfengPark/1_hdmap.bin",
        hdmap_path2 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/2_hdmap.bin",
        hdmap_path3 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/3_hdmap.bin",
        hdmap_path4 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/4_hdmap.bin",
        hdmap_path5 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/5_hdmap.bin",
        hdmap_path6 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/6_hdmap.bin",
        hdmap_path7 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/7_hdmap.bin",
        hdmap_path8 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/8_hdmap.bin",
        hdmap_path9 = "/home/shiny/"
                                    "/map/map_bin_files/qingfengPark/9_hdmap.bin",*/

        /// 2. Changzhou Industry
//          hdmap_path1 = "/home/shiny/map/map_bin_files/changzhou_industry/1_hdmap.bin",
//          hdmap_path2 = "/home/shiny/map/map_bin_files/changzhou_industry/2_hdmap.bin",
//          hdmap_path3 = "/home/shiny/map/map_bin_files/changzhou_industry/3_hdmap.bin",

        /// 3. Zhangqiu
//          hdmap_path1 = "/home/shiny/"
//                                      "/map/map_bin_files/zhangqiu/Scene_A/id1.bin",
//            hdmap_path2 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id2.bin",
//            hdmap_path3 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id3.bin",
//            hdmap_path4 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id4.bin",
//            hdmap_path5 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id5.bin",
//            hdmap_path6 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id6.bin",
//            hdmap_path7 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id7.bin",
//            hdmap_path8 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_A/id8.bin",
//            hdmap_path9 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id9.bin",
//            hdmap_path10 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id10.bin",
//            hdmap_path11 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id11.bin",
//            hdmap_path12 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id12.bin",
//            hdmap_path13 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id13.bin",
//            hdmap_path14 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id14.bin",
//            hdmap_path15 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id15.bin",
//            hdmap_path16 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id16.bin",
//            hdmap_path17 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id17.bin",
//            hdmap_path18 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_B/id18.bin",
//            hdmap_path19 = "/home/shiny/"
//                                        "/map/map_bin_files/zhangqiu/Scene_C/id19.bin",


        /// 4. Deyang
//          hdmap_path1 = "/home/shiny//map/map_bin_files/deyang/1223_hdmap.bin",


        /// 5. Chongqin
//          hdmap_path1 = "/home/shiny//map/map_bin_files/chongqin/1_hdmap.bin",
//          hdmap_path2 = "/home/shiny//map/map_bin_files/chongqin/2_hdmap.bin",
//          hdmap_path3 = "/home/shiny//map/map_bin_files/chongqin/3_hdmap.bin",

        /// 6. Shanghai
//          hdmap_path1 = "/home/shiny//map/map_bin_files/shanghai/1_hdmap.bin",


        /// 7. Zaozhuang scene_1
//          hdmap_path1 = "/home/shiny/maps/zaozhuang/scene_1/1_hdmap.bin",
//          hdmap_path2 = "/home/shiny/maps/zaozhuang/scene_1/2_hdmap.bin",
//          hdmap_path3 = "/home/shiny/maps/zaozhuang/scene_1/3_hdmap.bin",
//          hdmap_path4 = "/home/shiny/maps/zaozhuang/scene_1/4_hdmap.bin",



        ///8. zaozhuang scene_2
//          hdmap_path1 = "/home/shiny/maps/zaozhuang/scene_2/new/1_hdmap.bin",
//          hdmap_path2 = "/home/shiny/maps/zaozhuang/scene_2/2_hdmap.bin",
//          hdmap_path3 = "/home/shiny/maps/zaozhuang/scene_2/3_hdmap.bin",
//          hdmap_path4 = "/home/shiny/maps/zaozhuang/scene_2/4_hdmap.bin",
//          hdmap_path5 = "/home/shiny/maps/zaozhuang/scene_2/5_hdmap.bin",
//          hdmap_path6 = "/home/shiny/maps/zaozhuang/scene_2/6_hdmap.bin",
//          hdmap_path7 = "/home/shiny/maps/zaozhuang/scene_2/7_hdmap.bin",

        //9.shanghai
//          hdmap_path1="/home/shiny/maps/shanghai/1_hdmap.bin",

        //10.changfeng park
//         hdmap_path1="/home/shiny/maps/changfeng/0302/1_hdmap.bin",
//         hdmap_path2="/home/shiny/maps/changfeng/0302/2_hdmap.bin",
//        hdmap_path3="/home/shiny/maps/changfeng/0302/3_hdmap.bin",
//        hdmap_path4="/home/shiny/maps/changfeng/0302/4_hdmap.bin",
//hdmap_path1="/home/shiny/maps/changfeng/0309lidar/1_hdmap.bin",
//hdmap_path2="/home/shiny/maps/changfeng/0309lidar/2_hdmap.bin",
//hdmap_path3="/home/shiny/maps/changfeng/0309lidar/3_hdmap.bin",
//hdmap_path4="/home/shiny/maps/changfeng/0309lidar/4_hdmap.bin",
//          hdmap_path5="/home/shiny/maps/changfeng/0302/5_hdmap.bin",
//             hdmap_path1="/home/shiny/maps/changfeng/0224fail/1_hdmap.bin",
//changfeng single
//  hdmap_path1="/home/shiny/maps/changfeng/inner_single/1_hdmap.bin",

        //11.pingshan
//             hdmap_path1="/home/shiny/maps/pingshan/pingshantest/1_hdmap.bin",
//              hdmap_path1="/home/shiny/maps/pingshan/hdmap_pingshan0217(2).bin",

        //12.changzhou railway station
//           hdmap_path1="/home/shiny/maps/changzhou/1_hdmap.bin",
//           hdmap_path2="/home/shiny/maps/changzhou/2_hdmap.bin",
//           hdmap_path3="/home/shiny/maps/changzhou/3_hdmap.bin",
//           hdmap_path4="/home/shiny/maps/changzhou/4_hdmap.bin",



  //      13.baoan
//          hdmap_path1="/home/shiny/maps/baoan/HdmapBaoAn0301_no_terminal.bin",

//14.changzhou committee
//             hdmap_path1="/home/shiny/maps/changzhou/committee/1_hdmap.bin",

//15.changzhou factory
//          hdmap_path1="/home/shiny/maps/changzhou/factory/1_hdmap.bin",
//          hdmap_path2="/home/shiny/maps/changzhou/factory/2_hdmap.bin",

//16.wujinqu
           hdmap_path1="/home/shiny/maps/wujinqu/1_hdmap.bin",
           hdmap_path2="/home/shiny/maps/wujinqu/2_hdmap.bin",
           hdmap_path3="/home/shiny/maps/wujinqu/3_hdmap.bin",
           hdmap_path4="/home/shiny/maps/wujinqu/4_hdmap.bin",

//17.chongqin
//         hdmap_path1="/home/shiny/maps/chongqin/1_hdmap.bin",


        /// SIMULATION CONFIG PATH
//            config_path = "/home/shiny/zaozhuang/hdmapconfig.json";
//                       config_path = "/home/shiny/maps/zaozhuang/scene_1/hdmapconfig.json";
//               config_path = "/home/shiny/maps/zaozhuang/scene_2/new/hdmapconfig.json";
//                config_path = "/home/shiny/map/map_bin_files/changzhou_industry/hdmapconfig.json";
//          config_path = "/home/shiny/maps/shanghai/hdmapconfig.json";
//            config_path = "/home/shiny/maps/pingshan/pingshantest/hdmapconfig.json";
//            config_path = "/home/shiny/maps/changfeng/0302/hdmapconfig.json";
//            config_path = "/home/shiny/maps/changfeng/inner_single/hdmapconfig.json";
//        config_path = "/home/shiny/maps/changzhou/committee/hdmapconfig.json";
//       config_path = "/home/shiny/maps/changzhou/hdmapconfig.json";
// config_path = "/home/shiny/maps/changzhou/factory/hdmapconfig.json";
config_path = "/home/shiny/maps/wujinqu/hdmapconfig.json";

       //RECORD CONFIG PATH
//                 config_path ="/home/shiny/bug/0221changfeng/hdmapconfig.json";

//                config_path="/home/shiny/bug/0301baoan/hdmapconfig.json";

//                     config_path="/home/shiny/bug/0318changzhoufactory/hdmapconfig.json";
//  config_path="/home/shiny/bug/0315wujinqu/hdmapconfig.json";
//  config_path="/home/shiny/bug/0315wujinquafternoon/hdmapconfig.json";

///////////// RECORD PATH //////////////////
//std::string g_record_data_path = "/home/shiny/bug/0113zaozhuang/map_input.txt";
//std::string g_record_data_path = "/home/shiny/bug/0114zaozhuang/map_input.txt";
//std::string g_record_data_path = "/home/shiny/bug/0116deyang/map_input.txt";
//std::string g_record_data_path = "/home/shiny/bug/0211shanghai/map_input.txt";
//std::string g_record_data_path = "/home/shiny/bug/0213changfeng/map_input.txt";
//std::string g_record_data_path = "/home/shiny/bug/0221changfeng/map_input.txt";
    std::string g_record_data_path = "/home/shiny/bug/0318changzhoufactory/map_input.txt";
//std::string g_record_data_path = "/home/shiny/bug/0315wujinquafternoon/map_input.txt";


/////////////////////////////////CONFIG, MAP, RECORD PATH///////////////////////////////


////////////////////MAINWINDOW INITIALIZATION//////////////////
MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  this->setWindowTitle("PNC校验工具");
  string param_config_path = "/home/shiny/Planning/build/planconfig.json";
  string vehicle_config_path = "/home/shiny/Planning/build/vehicle_param.json";

  std_planning.Init( param_config_path,vehicle_config_path);

    hdmap_interface.MapInit(hdmap_path1, 1);
    hdmap_interface.MapInit(hdmap_path2, 1);
    hdmap_interface.MapInit(hdmap_path3, 1);
    hdmap_interface.MapInit(hdmap_path4, 1);
//    hdmap_interface.MapInit(hdmap_path5, 1);
//    hdmap_interface.MapInit(hdmap_path6, 1);
//    hdmap_interface.MapInit(hdmap_path7, 1);
//    hdmap_interface.MapInit(hdmap_path8, 1);
//    hdmap_interface.MapInit(hdmap_path9, 1);


  hdmap_interface.ConfigInit(config_path);



//  auto temp_map=hdmap_interface.GetAllMap();

//  auto temp_status=hdmap_interface.GetPncStatusString();

   config_mode_=PncInformation::Instance()->GetPncStatus().mode;

  auto current_file_load_status=PncInformation::Instance()->GetFileLoadStatusString();

  QPalette load_file_palette;
  if(current_file_load_status!="FILE_LOAD_OK"){
      load_file_palette.setColor(QPalette::Text, Qt::red);
  }
  else{
      load_file_palette.setColor(QPalette::Text, Qt::black);
  }
  ui->FileLoadStatusValue->setText(QString::fromStdString(current_file_load_status));
  ui->FileLoadStatusValue->setPalette(load_file_palette);

  ui->record_speed->setText("x"+QString::number(skip_times,'f',0));
  ui->record_speed->setAlignment(Qt::AlignCenter);

  connect(ui->pushButton,  SIGNAL(clicked(bool)),this,SLOT(Graph_Show()));
  connect(timer, SIGNAL(timeout()), this, SLOT(Graph_Show()));
//  connect(ui->Fast_Forward,SIGNAL(clicked(bool)),this,SLOT(on_Fast_Forward_clicked()));
//  connect(ui->Slow_Forward,SIGNAL(clicked(bool)),this,SLOT(on_Slow_Forward_clicked()));
}
////////////////////MAINWINDOW INITIALIZATION END//////////////////



////////////LOAD DATA////////////////
void Load_Map_Input(
        PublishData::MSFLoutput*msfl_output)
{
    static ifstream ifs(g_record_data_path, ios::in | ios::binary);
    if(ifs.is_open()){
        cout << "Read file open ok!!!" << endl;
    } else {
        PncInformation::Instance()->MutableGetPncStatus()->file_load_status=PncInformation::FileLoadStatus::RECORD_LOAD_ERROR;
        cout << "Read file open failed!!!" << endl;
    }

    boost::archive::binary_iarchive ia(ifs);

    ia >> *msfl_output;
}
////////////LOAD DATA END////////////////

void MainWindow::on_pushButton_clicked()
{

  if(!first) {
   button++;
  }else{
    first = false;
  }

}


void MainWindow::on_auto_2_clicked()
{
    if(!auto_go){
        timer->start(100);//ms
        auto_go = true;
        ui->auto_2->setText("stop");
      }else{
        ui->auto_2->setText("auto");
        if (timer->isActive() )
        timer->stop();
        auto_go = false;
      }

}

void MainWindow::on_Fast_Forward_clicked()
{
    if(skip_times<1024){
        skip_times*=2;
    }

   ui->record_speed->setText("x"+QString::number(skip_times,'f',0));
   ui->record_speed->setAlignment(Qt::AlignCenter);

}

void MainWindow::on_Slow_Forward_clicked()
{
    if(skip_times>1){
        skip_times/=2;
    }
   ui->record_speed->setText("x"+QString::number(skip_times,'f',0));
   ui->record_speed->setAlignment(Qt::AlignCenter);
}


MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::keyPressEvent(QKeyEvent *event){
  Q_UNUSED(event);
  if(event->key() == Qt::Key_Space)
    {
      if(!auto_go){
          timer->start(100);//ms
          auto_go = true;
          ui->auto_2->setText("stop");
        }else{
          ui->auto_2->setText("auto");
          if (timer->isActive() )
          timer->stop();
          auto_go = false;
        }
    }
}


void MainWindow::Graph_Show()
{

  Graph_Show(ui->widget);
}

void MainWindow::Graph_Show(QCustomPlot *CustomPlot)
{

    NewCurves(CustomPlot);
    NewRects(CustomPlot);
    NewTras(CustomPlot);
    NewIds(CustomPlot);
    NewHeadings(CustomPlot);
    NewLeftBoundary(CustomPlot);
    NewRightBoundary(CustomPlot);
    NewStations(CustomPlot);
    NewTerminals(CustomPlot);
    NewSites(CustomPlot);

  double t1,t2,t3,t4,t5,t6,t7, start_t;

  start_t = Clock::NowInMs();
  cout<<endl<<endl;
  if(auto_go) button++;
  cout<<"run_times = "<<run_times<<endl;


  static vector<TrajectoryPoint> trajectory_;
   PublishData::MSFLoutput msfl_output;
   unique_ptr<PublishData::MSFLoutput> msfl_output_ptr=make_unique<PublishData::MSFLoutput>();
  shared_ptr<PublishData::MSFLoutput>  msfl_shared_ptr=make_shared<PublishData::MSFLoutput>();
//   auto* msfl_output_ptr=&msfl_output;

  static bool start = true;
  static TrajectoryPoint pos;

  VehicleState vehicle_state;
  if(run_times > 0 && trajectory_.size() > 2){
      pos = trajectory_.at(2);
   }


//  if(hdmap_runtimes_==20){
//      start=true;
//  }

     if(config_mode_==PncInformation::Mode::SIMULATION_MODE){
        if(start){
        button = 50;
//     vehicle_state.x =10;//-3.5; // -2.; // -1.08;      ////zaozhuang
//     vehicle_state.y =  -20;//30.; // 64.; // 42.77;
//     vehicle_state.heading = 0;//3.14; // 3.; // -3.12;
//     vehicle_state.x =-136089; // -2.; // -1.08;      //// wujin error map
//     vehicle_state.y = 49132; // 64.; // 42.77;           ////
//     vehicle_state.heading = -1; // 3.; // -3.12;         ////
//     vehicle_state.x =-130; // -2.; // -1.08;      //// changfeng qiehuan
//     vehicle_state.y = -350; // 64.; // 42.77;           ////
//     vehicle_state.heading = 3.14; // 3.; // -3.12;         ////
//     vehicle_state.x =-16.8; // -2.; // -1.08;      ////changzhou factory
//     vehicle_state.y = -15; // 64.; // 42.77;           ////
//     vehicle_state.heading =0; // 3.; // -3.12;         ////
//          vehicle_state.x =0; // -2.; // -1.08;      //// changfeng new
//          vehicle_state.y = 0; // 64.; // 42.77;           ////
//          vehicle_state.heading =0; // 3.; // -3.12;         ////
          vehicle_state.x =0; // -2.; // -1.08;      ////wujinqu
          vehicle_state.y =0; // 64.; // 42.77;           ////
          vehicle_state.heading =-1; // 3.; // -3.12;         ////

//          if(hdmap_runtimes_>10){
//              vehicle_state.x =392; // -2.; // -1.08;      //// changfeng
//              vehicle_state.y =220; // 64.; // 42.77;           ////
//              vehicle_state.heading =0; // 3.; // -3.12;         ////
//          }
//          if(run_times<2000){
//              vehicle_state.x =1000; // -2.; // -1.08;      //// changfeng
//              vehicle_state.y =1000; // 64.; // 42.77;           ////
//              vehicle_state.heading =0; // 3.; // -3.12;         ////
//          }
//          else{
//              vehicle_state.x =1000; // -2.; // -1.08;      //// changfeng
//              vehicle_state.y =1000; // 64.; // 42.77;           ////
//              vehicle_state.heading =0; // 3.; // -3.12;         ////
//          }

//          vehicle_state.x =111; // -2.; // -1.08;      //// pingshan
//          vehicle_state.y =-497; // 64.; // 42.77;           ////
//          vehicle_state.heading =-1; // 3.; // -3.12;         ////
     vehicle_state.linear_velocity = 0;
     vehicle_state.linear_acceleration = 0;


     start = false;
   } else {
      //noise  x,y,theta,v
//    double random_x = random()%100 /3000.0 * pow(-1.0,random()%2);
//    double random_y = random()%100 /3000.0 * pow(-1.0,random()%2);
//    double ra_theta = random()%100 /100000.0 * pow(-1.0,random()%2);
//    double random_v = random()%100 /1000.0 * pow(-1.0,random()%2);
    vehicle_state.x = pos.path_point.x ;//+ random_x ;
    vehicle_state.y = pos.path_point.y ;//+ random_y;
    vehicle_state.heading = pos.path_point.theta ;//+ ra_theta;
    vehicle_state.linear_velocity = pos.v ;//+ random_v;
    vehicle_state.linear_acceleration = pos.a;
    vehicle_state.kappa =  pos.path_point.kappa ;

   }
        ui->VehicleState->setText(
                           "x = "  +QString::number(vehicle_state.x,'f',1)+
                 "<br/>" + "y = "  +QString::number(vehicle_state.y,'f',1)+
                 "<br/>" + "yaw = "+QString::number(vehicle_state.heading,'f',2)+
                 "<br/>" + "v = "  +QString::number(vehicle_state.linear_velocity,'f',1)+
                 "<br/>" + "kappa = "  +QString::number(vehicle_state.kappa,'f',4)
         );

     }
     else{

         for(int i=0;i<skip_times;i++){

             Load_Map_Input(msfl_output_ptr.get());
             msfl_output=*msfl_output_ptr.get();

         }
         run_times+=skip_times-1;

         hdmap::PointLLH current_LLH;
         current_LLH.lon=msfl_output.lon;
         current_LLH.lat=msfl_output.lat;
         current_LLH.height=msfl_output.height;
        auto current_map_point=hdmap::HDMapUtil::mapCoordintateTransform(current_LLH);

        vehicle_state.x=current_map_point.x;
        vehicle_state.y=current_map_point.y;
        auto current_heading=msfl_output.yaw;
        if(current_heading>0 && current_heading<180){
            current_heading=-current_heading;
        }
        else{
            current_heading=360-current_heading;
        }
        vehicle_state.heading=current_heading/180*M_PI;

         ui->VehicleState->setText(
                            "x = "  +QString::number(vehicle_state.x,'f',1)+
                  "<br/>" + "y = "  +QString::number(vehicle_state.y,'f',1)+
                  "<br/> "  +"lon = "+QString::number(msfl_output.lon ,'f',5)+
                  "<br/>" + "lat = "  +QString::number(msfl_output.lat,'f',5)+
                  "<br/>" + "yaw = "+QString::number(vehicle_state.heading,'f',2)
         );

         //////////////////////FOR RECORD LOAD STATUS////////////////////////
         auto current_file_load_status=PncInformation::Instance()->GetFileLoadStatusString();
         QPalette load_file_palette;
         if(current_file_load_status!="FILE_LOAD_OK"){
             load_file_palette.setColor(QPalette::Text, Qt::red);
         }
         else{
             load_file_palette.setColor(QPalette::Text, Qt::black);
         }
         ui->FileLoadStatusValue->setText(QString::fromStdString(current_file_load_status));
         ui->FileLoadStatusValue->setPalette(load_file_palette);
          //////////////////////FOR RECORD LOAD STATUS END////////////////////////
     }

  cout<<" vehicle_state(x,y,theta) = ("<<vehicle_state.x<<","
       <<vehicle_state.y<<","<<vehicle_state.heading<<")"<<endl;


///////////////////////////////VELOCITY AND KAPPA /////////////////////////
  static QVector<double> temp_x13, temp_y13,temp_y14;
  if(temp_y13.size()<120){
      temp_y13.push_front(vehicle_state.linear_velocity);
      temp_y14.push_front(vehicle_state.kappa * 100);

    }else{
      temp_y13.pop_back();
      temp_y14.pop_back();
      temp_y13.push_front(vehicle_state.linear_velocity);
      temp_y14.push_front(vehicle_state.kappa * 100);
    }
  if(run_times< 120){
      temp_x13.push_front(run_times);
    }
///////////////////////////////VELOCITY AND KAPPA  END/////////////////////////

  cout<<"/////////////////Mainwindow  map program start//////////////////////// "<<endl;

  hdmap::TrafficLightGroups traffic_light_groups;
  hdmap::PncRoutes pnc_routes;

  if(config_mode_==PncInformation::Mode::SIMULATION_MODE){
  msfl_output.lon = vehicle_state.x;
  msfl_output.lat = vehicle_state.y;
  msfl_output.gpsState = 5;
  msfl_output.navState = 7;
  double heading = vehicle_state.heading * 180 / M_PI;
  if (heading > 0)
      msfl_output.yaw = 360 - heading;
  else{
      msfl_output.yaw = -heading;
    }
  }

  hdmap::Point3D destination = {-20,7.5,0};
  vector<int> global_path;

  PublishData::Instruction instruction;
  instruction.use_instruction=true;
  instruction.selected_scene_id=1;
  instruction.selected_map_id=0;
//  map_runonce(msfl_output,destination,global_path,traffic_light_groups,pnc_routes);

  for(int i=0;i<5;i++){

  hdmap_interface.RunOnce(msfl_output,
                          destination,global_path,traffic_light_groups,pnc_routes);
}
//  static bool  temp_flag=true;
//  if(temp_flag){
//  hdmap_interface.SetInstruction(instruction);
//  temp_flag=false;
//}


  hdmap_runtimes_++;
  cout<<"hdmap runtimes :"<<hdmap_runtimes_<<endl;


  if (pnc_routes.empty()){
      cout << endl << endl << "pnc_routes is empty. " << endl << endl;
      return;
  }

  if (pnc_routes.front().is_update)
      cout << " pnc_routes.front() is updated"<<endl;

  cout<<"/////////////////Mainwindow  map program end////////////////////////"<<endl;




  std::unique_ptr<ReferenceLineProvider> line_provider1 =
                    std::make_unique<ReferenceLineProvider>(&pnc_routes);

  //line_provider1.get()->UpdateVehicleState(vehicle_state);
  std::list<ReferenceLine> reference_lines;
  std::list<hdmap::RouteSegment> routes;
  std::vector<hdmap::ReferenLineWithParam> ref_line_with_params;
  line_provider1.get()->GetReferenceLines(  &reference_lines,&routes,
                                            &ref_line_with_params);


  double overlap_start_x =0.0, overlap_start_y=0.0,
         overlap_end_x = 0.0,  overlap_end_y = 0.0;
  double start_x0, start_x1, start_y0,start_y1, end_x0, end_x1, end_y0, end_y1;
  hdmap::MapPoint overlap_start_map_point,overlap_end_map_point;


  auto station_ = reference_lines.front().MapRoute().GetBusStation();
  QVector<double> station_x,station_y;
  if(station_.id > 0){

      auto station_start =routes.front().GetMatchPoint(station_.start_s);
      auto station_end  = routes.front().GetMatchPoint(station_.end_s);

      station_x.push_back(station_start.point_enu.x);
      station_y.push_back(station_start.point_enu.y);
      station_x.push_back(station_end.point_enu.x);
      station_y.push_back(station_end.point_enu.y);

    }
  static QCPCurve * station = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  QPen Pen;
  Pen.setColor(Qt::green);
  Pen.setWidthF(5);
  station->data().reset();
  station->setPen(Pen);
  station->setData(station_x,station_y);
  station->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                             QPen(Qt::blue, 10), QBrush(Qt::white), 10 ) );

  auto traffic_light_ = reference_lines.front().MapRoute().SignalLightOverlaps();
  int traffic_id = 0;

  if(traffic_light_.has_traffic_light){
      traffic_id = traffic_light_.ids.front();
      auto start_s =   traffic_light_.start_s;
      auto end_s   =   traffic_light_.end_s;

      overlap_start_map_point =routes.front().GetMatchPoint(start_s);
      overlap_end_map_point  = routes.front().GetMatchPoint(end_s);
      overlap_start_x=overlap_start_map_point.point_enu.x;
      overlap_end_x=overlap_end_map_point.point_enu.x;
      overlap_start_y=overlap_start_map_point.point_enu.y;
      overlap_end_y=overlap_end_map_point.point_enu.y;
      auto overlap_start_theta=overlap_start_map_point.euler_angles.yaw;
      auto overlap_end_theta=overlap_start_map_point.euler_angles.yaw;
      start_x0=overlap_start_x+5.0*cos(overlap_start_theta);
      start_x1=overlap_start_x-5.0*cos(overlap_start_theta);
      start_y0=overlap_start_y+5.0*sin(overlap_start_theta);
      start_y1=overlap_start_y-5.0*sin(overlap_start_theta);
      end_x0=overlap_end_x+5.0*cos(overlap_end_theta);
      end_x1=overlap_end_x-5.0*cos(overlap_end_theta);
      end_y0=overlap_end_y+5.0*sin(overlap_end_theta);
      end_y1=overlap_end_y-5.0*sin(overlap_end_theta);

//      ui->trajectory_num->setText(" id : " +QString::number(traffic_id,'f',1) +
//                                  " start_s = " + QString::number(start_s,'f',1)
//                                  );
    }else{
//      ui->trajectory_num->setText("id : 0");
    }


  QVector<double> qoverlap_start_x,qoverlap_start_y,
                  qoverlap_end_x,qoverlap_end_y;
  if(traffic_id != 0)
  {
    qoverlap_start_x.push_back(start_x0);
    qoverlap_start_x.push_back(start_x1);
    qoverlap_start_y.push_back(start_y0);
    qoverlap_start_y.push_back(start_y1);
    qoverlap_end_x.push_back(end_x0);
    qoverlap_end_x.push_back(end_x1);
    qoverlap_end_y.push_back(end_y0);
    qoverlap_end_y.push_back(end_y1);
  }

  QVector<double> tempx1 ,  tempy1,  temp_x0, temp_y0, temp_x01, temp_y01 ,
                  temp_x02 ,temp_y02,temp_x03,temp_y03,temp_x12, temp_y12,
                  temp_x22 ,temp_y22;


  QVector<double> tempX, tempY;   // -- FOR X, Y OF POINTS OF ALL MAPS other than current map
  QVector<double> tempX1, tempY1;   // -- FOR X, Y OF POINTS OF THE MAP BEING NAVIGATED
  QVector<double> global_station_x,global_station_y;//single station start point and end point
  QVector<pair<QVector<double>,QVector<double>>> global_stations;//total stations
  QVector<double> global_terminal_x,global_terminal_y;//single station start point and end point
  QVector<pair<QVector<double>,QVector<double>>> global_terminals;//total stations
//  QVector<double> global_site_x,global_site_y;//single station start point and end point //to be added
//  QVector<pair<QVector<double>,QVector<double>>> global_sites;//total stations

  //////////////////////////////////GLOBAL MAPS ATTRIBUTES///////////////////
  for (vector<const hdmap::HDMap *> hdmap_pointers : hdmap_interface.maps_){
      for (const hdmap::HDMap * hdmap_pointer : hdmap_pointers){
          if (hdmap_pointer != hdmap_interface.map_){
              for(hdmap::Lane lane : hdmap_pointer->lanes){
                  for (hdmap::MapPoint map_point : lane.central_points){
                        tempX.push_back(map_point.point_enu.x);
                        tempY.push_back(map_point.point_enu.y);
                  }

              }
          }
          for(auto current_lane:hdmap_pointer->lanes){
              ///global station start////
              if(current_lane.station.id!=-1){
              auto current_station_start_point=GetLaneMatchPoint(current_lane,current_lane.station.start_s);
              auto current_station_end_point=GetLaneMatchPoint(current_lane,current_lane.station.end_s);

              global_station_x.clear();
              global_station_y.clear();
              global_station_x.push_back(current_station_start_point.point_enu.x);
              global_station_y.push_back(current_station_start_point.point_enu.y);
              global_station_x.push_back(current_station_end_point.point_enu.x);
              global_station_y.push_back(current_station_end_point.point_enu.y);

              global_stations .push_back(std::make_pair(global_station_x,global_station_y));

              }
             ///global station end////

              ///global terminal start////
              if(current_lane.terminal.id!=-1){
              auto current_terminal_start_point=GetLaneMatchPoint(current_lane,current_lane.terminal.start_s);
              auto current_terminal_end_point=GetLaneMatchPoint(current_lane,current_lane.terminal.end_s);

               global_terminal_x.clear();
               global_terminal_y.clear();
               global_terminal_x.push_back(current_terminal_start_point.point_enu.x);
               global_terminal_y.push_back(current_terminal_start_point.point_enu.y);
               global_terminal_x.push_back(current_terminal_end_point.point_enu.x);
               global_terminal_y.push_back(current_terminal_end_point.point_enu.y);

               global_terminals .push_back(std::make_pair(global_terminal_x,global_terminal_y));

              }
              ///global terminal end////

              ///global sites start//// to be added
//              if(current_lane.site.id!=-1){
//              auto current_site_start_point=GetLaneMatchPoint(current_lane,current_lane.site.start_s);
//              auto current_site_end_point=GetLaneMatchPoint(current_lane,current_lane.site.end_s);

//              global_site_x.clear();
//               global_site_y.clear();
//               global_site_x.push_back(current_site_start_point.point_enu.x);
//               global_site_y.push_back(current_site_start_point.point_enu.y);
//               global_site_x.push_back(current_site_end_point.point_enu.x);
//               global_site_y.push_back(current_site_end_point.point_enu.y);

//               global_sites .push_back(std::make_pair(global_site_x,global_site_y));

//              }
              ///global sites end////
      }
    }
  }

  /// CURRENT MAP
  for(hdmap::Lane lane : hdmap_interface.map_->lanes){
      for (hdmap::MapPoint map_point : lane.central_points){
            tempX1.push_back(map_point.point_enu.x);
            tempY1.push_back(map_point.point_enu.y);
      }
  }



  ///////////////////Visualization of global station,terminal,site in the whole maps start////////////////////////
  QPen station_pen;
  station_pen.setColor(Qt::blue);
  station_pen.setWidthF(3);

  for(int i=0;i<global_stations.size();i++){
      qcp_stations_ptrs_[i]->data().reset();
      qcp_stations_ptrs_[i]->setPen(station_pen);
      qcp_stations_ptrs_[i]->setData(global_stations[i].first,global_stations[i].second);
      qcp_stations_ptrs_[i]->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                                                             QPen(Qt::blue,10),QBrush(Qt::white),10));
  }

  QPen terminal_pen;
  terminal_pen.setColor(Qt::red);
  terminal_pen.setWidthF(3);

  for(int i=0;i<global_terminals.size();i++){
      qcp_terminals_ptrs_[i]->data().reset();
      qcp_terminals_ptrs_[i]->setPen(terminal_pen);
      qcp_terminals_ptrs_[i]->setData(global_terminals[i].first,global_terminals[i].second);
      qcp_terminals_ptrs_[i]->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                                                             QPen(Qt::red,10),QBrush(Qt::white),10));
}
//  QPen site_pen;//to be added
//  site_pen.setColor(Qt::green);
//  site_pen.setWidthF(5);

//  for(int i=0;i<global_sites.size();i++){
//      qcp_sites_ptrs_[i]->data().reset();
//      qcp_sites_ptrs_[i]->setPen(site_pen);
//      qcp_sites_ptrs_[i]->setData(global_sites[i].first,global_sites[i].second);
//      qcp_sites_ptrs_[i]->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
//                                                             QPen(Qt::blue,10),QBrush(Qt::white),10));
//      }
  ///////////////////Visualization of global station,terminal,site in the whole maps end////////////////////////

  //////////////////////////////////GLOBAL MAPS ATTRIBUTES END///////////////////

QVector<double> temp_route_left_boundary_x,
        temp_route_left_boundary_y,temp_route_right_boundary_x,temp_route_right_boundary_y;


  QVector<QVector<double>> route_left_boundary_x_vec,route_left_boundary_y_vec,
                                                           route_right_boundary_x_vec,route_right_boundary_y_vec;

  //1.map reference line
  PathPoint path_point;

  for(auto map_route:routes) {
      auto route = map_route.GetRoute();

      std::pair<vector<PathPoint>,vector<PathPoint>> temp_route_boundary_pair;


      for(auto &temp_reference_line:reference_lines){
          if(route.id==temp_reference_line.GetReferenceLineId()){
                temp_route_boundary_pair= SetBoundaryLanePoints(route.reference_points,&temp_reference_line);
          }
      }
      temp_route_left_boundary_x.clear();
      temp_route_left_boundary_y.clear();
      temp_route_right_boundary_x.clear();
      temp_route_right_boundary_y.clear();

      for(int i = 0; i < route.reference_points.size() ; i++){
          auto x2 = route.reference_points.at(i).point_enu.x;
          auto y2 = route.reference_points.at(i).point_enu.y;
          auto theta2 = route.reference_points.at(i).euler_angles.yaw;
          auto theta = theta2 * 180.0/M_PI;
          tempx1.push_back(x2) ;
          tempy1.push_back(y2);
          temp_x22.push_back( i );
          temp_y22.push_back( theta);
      }

      for(int i=0;i<temp_route_boundary_pair.first.size();i++) {

          auto route_left_boundary_x=temp_route_boundary_pair.first.at(i).x;
          auto route_left_boundary_y=temp_route_boundary_pair.first.at(i).y;
          auto route_right_boundary_x=temp_route_boundary_pair.second.at(i).x;
          auto route_right_boundary_y=temp_route_boundary_pair.second.at(i).y;
          temp_route_left_boundary_x.push_back(route_left_boundary_x);
          temp_route_left_boundary_y.push_back(route_left_boundary_y);
          temp_route_right_boundary_x.push_back(route_right_boundary_x);
          temp_route_right_boundary_y.push_back(route_right_boundary_y);
  }


      route_left_boundary_x_vec.push_back(temp_route_left_boundary_x);
      route_left_boundary_y_vec.push_back(temp_route_left_boundary_y);
      route_right_boundary_x_vec.push_back(temp_route_right_boundary_x);
      route_right_boundary_y_vec.push_back(temp_route_right_boundary_y);

  }
///11111111111111111111111111111111111111111111111111111111111111111111
    std::vector<hdmap::MapPoint> anchor_points;
    line_provider1.get()->GetAnchorPoints(&routes.front(), &anchor_points);
    for(int i = 0 ; i<anchor_points.size(); i ++ ){

        auto x2 = anchor_points.at(i).point_enu.x;
        auto y2 = anchor_points.at(i).point_enu.y;
        //cout<<"point1 ("<<i<<")"<<x2<<","<<y2<<endl;
        temp_x0.push_back( x2 );
        temp_y0.push_back( y2 );
      }
///2222222222222222222222222222222222222222222222222222222222222

    for(int i = 0 ;i < reference_lines.front().reference_points().size() ;i++)
      {
        auto x2 = reference_lines.front().reference_points().at(i).x;
        auto y2 = reference_lines.front().reference_points().at(i).y;
        auto theta = reference_lines.front().reference_points().at(i).theta/M_PI *180.0;
        auto kappa = reference_lines.front().reference_points().at(i).kappa;
        auto dkappa = reference_lines.front().reference_points().at(i).dkappa;
        auto s = reference_lines.front().reference_points().at(i).s;
        temp_x02.push_back( i );
        temp_y02.push_back( theta);
        temp_x03.push_back( i );
        temp_y03.push_back( kappa *100);
        temp_x01.push_back( x2 );
        temp_y01.push_back( y2 );
        temp_x12.push_back( i );
        temp_y12.push_back( s + 50);
      }

    t1 = Clock::NowInMs() - start_t;
    start_t = Clock::NowInMs();

///location//////////////////////////////////////////////////////////////////
    localization::LocalizationEstimate location_;
    Pose pose;
    pose.timestamp_ms = Clock::NowInMs();
    pose.position.x = vehicle_state.x;
    pose.position.y = vehicle_state.y;
    pose.euler_angles.z = vehicle_state.heading;
    pose.linear_velocity.y = vehicle_state.linear_velocity *
                            sin(vehicle_state.heading + M_PI_2);
    pose.linear_velocity.x = vehicle_state.linear_velocity *
                            cos(vehicle_state.heading + M_PI_2);
    pose.linear_acceleration_vrf.y = vehicle_state.linear_acceleration;
    pose.gps_status = RTK_FIXATION;
    pose.is_nav = true;
    location_.pose_30.push_back(pose);
///33333333333333333333333333333333333333333333333333333333333
  prediction::PredictionObstacles obstacles_;
  //obstacles_.header.timestamp_ms = Clock::NowInMs();
  obstacles_.start_timestamp = Clock::NowInMs();
//  auto obs1_= CreatObstacle(1, 20.0,20.0,0.75,
//                            0.0,0.0, 7.0,3.0,2.0, 4, 0);
//  obstacles_.prediction_obstacle.push_back(obs1_);

  auto obs2_ = CreatObstacle(2,-3.5,-16,0.55,
                             0.0 - pose.linear_velocity.x,0.0- pose.linear_velocity.y,
                             6.0,4.0,4.0,
                             4,0,vehicle_state);
//  obstacles_.prediction_obstacle.push_back(obs2_);

  auto obs3_ = CreatObstacle(3,-2.5,-25,0.8,
                              0.0 - pose.linear_velocity.x,0.0- pose.linear_velocity.y,
                             4,2,4.0,
                             4,0,vehicle_state);
//  obstacles_.prediction_obstacle.push_back(obs3_);

  auto obs4_ = CreatObstacle(4,-117,-80,-0.5,
                              0.0 - pose.linear_velocity.x,0.0- pose.linear_velocity.y,
                             6.0,4.0,4.0,
                             4,0,vehicle_state);
  obstacles_.prediction_obstacle.push_back(obs4_);
  auto obs6_ = CreatObstacle(6,-3,40,-0.5,
                              0.0 - pose.linear_velocity.x,0.0- pose.linear_velocity.y,
                             3.0,3.0,4.0,
                             4,0,vehicle_state);
  obstacles_.prediction_obstacle.push_back(obs6_);
  auto obs7_ = CreatObstacle(7,-10,55,-0.5,
                              0.0 - pose.linear_velocity.x,0.0- pose.linear_velocity.y,
                             1,2,4.0,
                             4,0,vehicle_state);
  obstacles_.prediction_obstacle.push_back(obs7_);
  int jump_ = run_times%10;
  if(jump_ <= 7){

     double speed = run_times%330;
     double rate = 0.1,
            v_x  = 0.8 ,v_y = 2;
     auto p_x = v_x *speed * rate ;
     auto p_y = v_y *speed * rate ;
     auto obs5_ = CreatObstacle(5,102.5 + p_x ,-242.5 + p_y,1.2,
                                v_x - pose.linear_velocity.x ,
                                v_y - pose.linear_velocity.y ,
                                6.0,2.0,4.0,
                                4,1,vehicle_state);
     obstacles_.prediction_obstacle.push_back(obs5_);
  }else{
     int a = 0;
  }
  obstacles_.prediction_obstacle.clear();
  obstacles_.prediction_obstacle_num = obstacles_.prediction_obstacle.size();
  obstacles_.end_timestamp = Clock::NowInMs();

  ///chassis//////////////////////////////////////////////////////////////////
  canbus::Chassis chassis_;
  chassis_.timestamp_ms = Clock::NowInMs();
  chassis_.speed_mps = vehicle_state.linear_velocity;
  chassis_.driving_mode = COMPLETE_AUTO_DRIVE;

  int color_type = run_times%600;
  perception::TrafficLightDetection traffic_;
  perception::TrafficLight light_;
  traffic_.camera_timestamp = Clock::NowInMs();
  traffic_.frame_num = 1;
//  if(color_type<50){
//     light_.color = perception::YELLOW;
//     ui->traffic_light->setText("YELLOW");
//   }else if(color_type<350){
//     light_.color = perception::RED;
//     ui->traffic_light->setText("RED");
//   }else{
//     light_.color = perception::GREEN;
//     ui->traffic_light->setText("GREEN");
//   }
  light_.id = 11;
  traffic_.traffic_light.push_back(light_);
///************set garbages data **********************************//

  CleanTarget clean_target_;
  clean_target_.timestamp = Clock::NowInMs();
  Garbage g0;
//  g0.id = 1;
//  g0.position.x = -38;
//  g0.position.y = -10;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 2;
//  g0.position.x = -36;
//  g0.position.y = -20;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 3;
//  g0.position.x = -39;
//  g0.position.y = 0;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 4;
//  g0.position.x = -39.5;
//  g0.position.y = 1.5;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 5;
//  g0.position.x = -37.5;
//  g0.position.y = 0.5;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 6;
//  g0.position.x = 142.5;
//  g0.position.y = -110;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 7;
//  g0.position.x = 140;
//  g0.position.y = -110;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 8;
//  g0.position.x = 140;
//  g0.position.y = -100;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 9;
//  g0.position.x = 140.5;
//  g0.position.y = -100;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 10;
//  g0.position.x = 120;
//  g0.position.y = -70;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 11;
//  g0.position.x = 135;
//  g0.position.y = -90;
//  clean_target_.Garbages.push_back(g0);
//  g0.id = 12;
//  g0.position.x = 119;
//  g0.position.y = -73;
//  clean_target_.Garbages.push_back(g0);

  GetCarCoordinateCleanData(clean_target_,vehicle_state);


///*********************ultrasonic sense *****************************/

  UltrasonicSense ultrasonic_;
  Ultrasonic ult_1;
  ult_1.id = 1;
  ult_1.valid = false;
  ult_1.distance = 0.5;
  ultrasonic_.front.push_back(ult_1);
  ultrasonic_.time_stamp = Clock::NowInMs();
///444444444444444444444444444444444444444444444444444444444444444
  cout<<"************************************************"<<endl<<endl;
  cout<<"planning start .."<<endl;
  start_t = Clock::NowInMs();
  //SoftCommand   command_;
//  if(park_mode%2 == 1){
//      ui->ParkingMode->setText("EnAutoPark");
//      command_.auto_parking = true;
//   }else{
//      ui->ParkingMode->setText("UnAutoPark");
//      command_.auto_parking = false;
//   }

//  if(clean_mode%3 == 0){
//      ui->CleanMode->setText("DriveMode");
//      command_.clean_mode = DRIVE_MODE;
//  }else if(clean_mode%3 == 1){
//      ui->CleanMode->setText("ShowMode");
//      command_.clean_mode = SHOW_MODE;

//  }else{
//      ui->CleanMode->setText("BorderMode");
//      command_.clean_mode = BORDER_MODE;
//  }
  //hdmap::PncRoutes             pnc_routes_;



/////////////////////// Planning Module call ///////////////////////////


  PbTrajectory pb_trajectory_ =
      std_planning.RunOnce(&obstacles_,&location_,    &chassis_,
                           &traffic_,  &clean_target_,&ultrasonic_,&command_,
                           &pnc_routes);
  trajectory_ = pb_trajectory_.trajectory_points;


////////////////////////////////////////////////////////////////////////




  auto start_point = pose.position;//location_.pose_30.back().position;

  ////////////////////////////////
    // assign value(x,y) to start_point



  //////////////////////////////////



///55555555555555555555555555555555555555555555555555555555555555555
  QVector<double> temp_x5 , temp_y5,temp_x6 , temp_y6,
                  temp_x7 , temp_y7,temp_x8 , temp_y8,
                  temp_x9 , temp_y9,temp_x10 , temp_y10,
                  temp_x11 , temp_y11;

  for(int i = 0 ; i < trajectory_.size() ;i++)
    {
      temp_x5.push_back(trajectory_.at(i).path_point.x);
      temp_y5.push_back(trajectory_.at(i).path_point.y);
      temp_x6.push_back(i+ 50);
      temp_y6.push_back(trajectory_.at(i).path_point.theta *180.0 /M_PI);
      temp_x7.push_back(i+ 50);
      temp_y7.push_back(trajectory_.at(i).path_point.kappa * 100);
      temp_x8.push_back(i);
      temp_y8.push_back(trajectory_.at(i).v -50);
      temp_x10.push_back(i);
      temp_y10.push_back(trajectory_.at(i).path_point.s + 20);
    }
  t2 = Clock::NowInMs() - start_t;
  start_t = Clock::NowInMs();
#if 1


 ///////////////////////////////// Visualization ///////////////////////////////////////


  static QCPCurve * curve_000 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_000->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                              QPen(Qt::green, 3), QBrush(Qt::green), 2));
  curve_000->setLineStyle( QCPCurve::LineStyle(QCPGraph::lsNone));
  curve_000->data().reset();
  curve_000->setData(tempX,tempY);
  t3 = Clock::NowInMs() - start_t;
  start_t = Clock::NowInMs();

  //////////////////current map  center points////////////////////////////
  static QCPCurve * curve_001 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_001->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                              QPen(Qt::yellow, 3), QBrush(Qt::yellow), 2));
  curve_001->setLineStyle( QCPCurve::LineStyle(QCPGraph::lsNone));
  curve_001->data().reset();
  curve_001->setData(tempX1,tempY1);
  t4 = Clock::NowInMs() - start_t;
  start_t = Clock::NowInMs();
 //////////////////current map center points end////////////////////////////

    //////////////////current route  center points////////////////////////////
  static QCPCurve * curve_00 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_00->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                              QPen(Qt::blue, 3), QBrush(Qt::white), 2));
  curve_00->setLineStyle( QCPCurve::LineStyle(QCPGraph::lsNone));
  curve_00->data().reset();
  curve_00->setData(tempx1,tempy1);
  t5 = Clock::NowInMs() - start_t;
  start_t = Clock::NowInMs();
  //////////////////current route  center points////////////////////////////

  ////////////////current referenceline anchor points///////////////////
  static QCPCurve * curve_01 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_01->setPen(QPen(Qt::black));
  curve_01->setLineStyle( QCPCurve::LineStyle(QCPGraph::lsNone));
  curve_01->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                              QPen(Qt::darkGray, 2), QBrush(Qt::white), 10));
  curve_01->data().reset();
  curve_01->setData(temp_x0,temp_y0);
  t6 = Clock::NowInMs() - start_t;
  start_t = Clock::NowInMs();
  ////////////////current referenceline anchor points end ///////////////////

  ///////////////current reference points//////////////////////////////////
  static QCPCurve * curve_02 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_02->setPen(QPen(Qt::black));
  curve_02->setLineStyle(QCPCurve::LineStyle(QCPGraph::lsNone));
  curve_02->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                              QPen(Qt::black, 1), QBrush(Qt::white), 2));
  curve_02->data().reset();
  curve_02->setData(temp_x01,temp_y01);
  t7 = Clock::NowInMs() - start_t;
  start_t = Clock::NowInMs();
    ///////////////current reference points end//////////////////////////////////

  //////////////////////////////referenceline theta curve start///////////////////////////////////
//  static QCPCurve * curve_03 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_03->data().reset();
//  curve_03->setPen(QPen(Qt::blue));
//  curve_03->setLineStyle(QCPCurve::LineStyle(QCPGraph::lsNone));
//  curve_03->setData(temp_x02,temp_y02);
//  curve_03->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
//                              QPen(Qt::black, 1), QBrush(Qt::white), 5));
  ///////////////////////////////referenceline theta curve end////////////////////////////////////
  /// \brief curve_33
  //////////////////route theta//////////////////////////////
//  static QCPCurve * curve_33 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_33->data().reset();
//  curve_33->setPen(QPen(Qt::blue));
//  curve_33->setLineStyle(QCPCurve::LineStyle(QCPGraph::lsNone));
//  curve_33->setData(temp_x22,temp_y22);
//  curve_33->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
//                              QPen(Qt::red, 1), QBrush(Qt::white), 5));
    //////////////////route theta end//////////////////////////////

  ///////////////////////referenceline kappa curve start//////////////////////////
//  static QCPCurve * curve_04 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_04->data().reset();
//  curve_04->setPen(QPen(Qt::green));
//  curve_04->setData(temp_x03,temp_y03);
//  curve_04->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
//                              QPen(Qt::black, 1), QBrush(Qt::white), 1));
  ///////////////////////////////referenceline kappa curve end//////////////////////

  /////////////// //current trajectory position///////////////////////////////////////
if(config_mode_==PncInformation::Mode::SIMULATION_MODE){
  static QCPCurve * curve_05 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_05->data().reset();
  curve_05->setPen(QPen(Qt::black));
  curve_05->setData(temp_x5,temp_y5);
  curve_05->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                              QPen(Qt::red, 1), QBrush(Qt::red), 5));
}
   /////////////// //current trajectory position end///////////////////////////////////////

  ///////////////////trajectory points heading//////////////////////
//  static QCPCurve * curve_06 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_06->data().reset();
//  curve_06->setPen(QPen(Qt::red));
//  curve_06->setData(temp_x6,temp_y6);
//////////////////trajectory points heading end/////////////////////

  ////////////////trajectory points kappa/////////////////////////
//  static QCPCurve * curve_07 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_07->data().reset();
//  curve_07->setPen(QPen(Qt::red));
//  curve_07->setData(temp_x7,temp_y7);
    ////////////////trajectory points kappa end/////////////////////////

    ////////////////trajectory v /////////////////////////
//  static QCPCurve * curve_08 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_08->data().reset();
//  curve_08->setPen(QPen(Qt::red));
//  curve_08->setData(temp_x8,temp_y8);
   ////////////////trajectory v end/////////////////////////

  /////////////////////route widths visualization////////////////////////////
  QPen bound_pen;
  bound_pen.setColor(Qt::lightGray);
  bound_pen.setStyle(Qt::SolidLine);
  bound_pen.setWidth(5);
  for(int i=0;i<route_left_boundary_x_vec.size();i++){

      qcp_left_lane_boudary_ptrs_.at(i)->data().reset();
      qcp_left_lane_boudary_ptrs_.at(i)->setPen(bound_pen);
      qcp_left_lane_boudary_ptrs_.at(i)->setData(route_left_boundary_x_vec[i],route_left_boundary_y_vec[i]);

  }

  for(int i=0;i<route_right_boundary_x_vec.size();i++){

      qcp_right_lane_boudary_ptrs_.at(i)->data().reset();
      qcp_right_lane_boudary_ptrs_.at(i)->setPen(bound_pen);
      qcp_right_lane_boudary_ptrs_.at(i)->setData(route_right_boundary_x_vec[i],route_right_boundary_y_vec[i]);

  }

  ////////////////////route widths visualization end/////////////////////////

  temp_x9.clear();
  temp_y9.clear();
  temp_x8.clear();
  temp_y8.clear();
  temp_x9.push_back(start_point.x);
  temp_y9.push_back(start_point.y);
  if(1){
  static QCPItemText *textLabel = new QCPItemText(CustomPlot);
  textLabel->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
  //textLabel->position->setType(QCPItemPosition::ptAxisRectRatio);
  textLabel->position->setCoords(start_point.x,
                                 start_point.y);
  textLabel->setText(QString::number(0,'f',0));
  textLabel->setFont(QFont(font().family(), 16)); // make font a bit larger
  //textLabel->deleteLater();
  //textLabel->setPen(QPen(Qt::black)); // show black border around text

  // add the arrow:
  //auto arrow = make_shared<QCPItemLine>(CustomPlot);
  static QCPItemLine* arrow = new QCPItemLine(CustomPlot);

  arrow->start->setParentAnchor(textLabel->position);
  double arrow_length = 10;
  auto x = start_point.x +
           arrow_length * cos(vehicle_state.heading + M_PI_2);
  auto y = start_point.y +
           arrow_length * sin(vehicle_state.heading + M_PI_2);
  arrow->end->setCoords(x, y); // point to (4, 1.6) in x-y-plot coordinates
  arrow->setHead(QCPLineEnding::esSpikeArrow);
  //arrow->deleteLater();
  }
  static QCPCurve *fermatSpiral1 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);

  for(int i = 0;i<4;i++ )
   {

      Point3D point = {0,0,0} ;
      if(i<2){
          point.x += g_vehicle_config.width/2.0;
         if(i%2 == 0)
          point.y += g_vehicle_config.front_edge_to_center;
         else
          point.y -= g_vehicle_config.back_edge_to_center;
       }else{
          point.x -= g_vehicle_config.width/2.0;
          if(i%2 == 0)
           point.y -= g_vehicle_config.back_edge_to_center;
          else
           point.y += g_vehicle_config.front_edge_to_center;
       }

      CoordinateConvert::PointRelativeTranslationAndRotaion(point,start_point,
                                                 -pose.euler_angles.z);
      temp_x8.push_back(point.x);
      temp_y8.push_back(point.y);
   }

  temp_x8.push_back(temp_x8.front());
  temp_y8.push_back(temp_y8.front());
  fermatSpiral1->setData(temp_x8,temp_y8);

  static QCPCurve * curve_09 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_09->data().reset();
  curve_09->setPen(QPen(Qt::red));
  curve_09->setData(temp_x9,temp_y9);
  curve_09->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssStar,
                            QPen(Qt::blue, 2), QBrush(Qt::white), 8 ) );


  ////////////////////////////////////trajectory s////////////////////////////
//  static QCPCurve * curve_10 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_10->data().reset();
//  curve_10->setPen(QPen(Qt::red));
//  curve_10->setData(temp_x10,temp_y10);
//  curve_10->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssStar,
//                            QPen(Qt::blue, 2), QBrush(Qt::white), 5 ) );
  ////////////////////////////////////trajectory s end////////////////////////////

  QVector<double> garbages_x,garbages_y;
  auto clean_target = AdapterManager::GetCleanTarget();
  for(auto garbage : clean_target.Garbages ){

      garbages_x.push_back(garbage.position.x);
      garbages_y.push_back(garbage.position.y);
   }
  ///****************************************************************************
  /// \brief garbage_show
  ///****************************************************************************
  static QCPCurve * garbage_show = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  QPen Pen1;
  Pen1.setColor(Qt::green);
  Pen1.setWidthF(5);
  garbage_show->data().reset();
  garbage_show->setPen(Pen1);
  garbage_show->setLineStyle( QCPCurve::LineStyle(QCPGraph::lsNone));
  garbage_show->setData(garbages_x,garbages_y);
  garbage_show->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                             QPen(Qt::blue, 10), QBrush(Qt::white), 10 ) );

///show obstacle *****************************************************************///
  prediction::PredictionObstacles obstacles = *AdapterManager::GetLatestObserved();

  for(int i = 0 ; i <64;i++){
      qcp_ploygon_ptrs_.at(i)->setVisible(false);
      qcp_rect_ptrs_.at(i)->setVisible(false);
      qcp_tra_ptrs_.at(i)->setVisible(false);
      qcp_id_ptrs_.at(i)->setVisible(false);
      qcp_heading_ptrs_.at(i)->setVisible(false);
  }

  for(int i = 0 ; i < obstacles.prediction_obstacle_num &&
      obstacles.prediction_obstacle_num < 64; i++){

      auto obs_ = obstacles.prediction_obstacle.at(i);
      //1. draw obstacle ploygon
      temp_x11.clear();
      temp_y11.clear();
      for(int j = 0 ; j < obs_.perception_obstacle.polygon_point.size(); j++){
         temp_x11.push_back(obs_.perception_obstacle.polygon_point.at(j).x);
         temp_y11.push_back(obs_.perception_obstacle.polygon_point.at(j).y);
      }
      temp_x11.push_back(obs_.perception_obstacle.polygon_point.at(0).x);
      temp_y11.push_back(obs_.perception_obstacle.polygon_point.at(0).y);

      qcp_ploygon_ptrs_.at(i)->setVisible(true);
      qcp_ploygon_ptrs_.at(i)->data().reset();
      qcp_ploygon_ptrs_.at(i)->setData(temp_x11,temp_y11);

      //2. draw obstacle trajectory
      temp_x11.clear();
      temp_y11.clear();
      if(obs_.trajectory.size() > 0 ){
        auto tra = obs_.trajectory.front();
        for(int k = 0 ; k < tra.trajectory_point.size();k++){
            temp_x11.push_back(tra.trajectory_point.at(k).path_point.x);
            temp_y11.push_back(tra.trajectory_point.at(k).path_point.y);
         }
      }

      qcp_tra_ptrs_.at(i)->data().reset();
      qcp_tra_ptrs_.at(i)->setVisible(true);
      qcp_tra_ptrs_.at(i)->setData(temp_x11,temp_y11);
      qcp_tra_ptrs_.at(i)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                                    QPen(Qt::blue, 5), QBrush(Qt::white), 4 ));
      //3. draw obstacle id
      qcp_id_ptrs_.at(i)->setVisible(true);
      qcp_id_ptrs_.at(i)->setPositionAlignment(Qt::AlignTop|Qt::AlignHCenter);
      qcp_id_ptrs_.at(i)->position->setCoords(obs_.perception_obstacle.position.x,
                                     obs_.perception_obstacle.position.y);
      qcp_id_ptrs_.at(i)->setText(QString::number(obs_.perception_obstacle.id,'f',0));
      qcp_id_ptrs_.at(i)->setFont(QFont(font().family(), 16)); // make font a bit larger

      //4.draw obstacle the arrow:
      qcp_heading_ptrs_.at(i)->setVisible(true);
      qcp_heading_ptrs_.at(i)->start->setParentAnchor( qcp_id_ptrs_.at(i)->position);
      double arrow_length = 10;
      auto x = obs_.perception_obstacle.position.x +
               arrow_length * cos(obs_.perception_obstacle.theta + M_PI_2);
      auto y = obs_.perception_obstacle.position.y +
               arrow_length * sin(obs_.perception_obstacle.theta + M_PI_2);
      qcp_heading_ptrs_.at(i)->end->setCoords(x, y); // point to (4, 1.6) in x-y-plot coordinates
      qcp_heading_ptrs_.at(i)->setHead(QCPLineEnding::esSpikeArrow);

  }

///***********draw obstacles end ***********************************************///



  ////////////////////////////referenceline s (shift) curve start///////////////////////
//  static QCPCurve * curve_11 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_11->data().reset();
//  curve_11->setPen(QPen(Qt::blue));
//  curve_11->setData(temp_x12,temp_y12);
  ////////////////////////////referenceline s (shift) curve end///////////////////////


  ////////////////////////////////vehicle state velocity////////////////////////////
//  static QCPCurve * curve_13 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_13->data().reset();
//  curve_13->setPen(QPen(Qt::blue));
//  curve_13->setData(temp_x13,temp_y13);
//  curve_13->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
//                             QPen(Qt::red, 3), QBrush(Qt::white), 5 ) );
  ////////////////////////////////vehicle state velocity end////////////////////////////
  /// \brief curve_14
  ///////////////////////////////vehicle state kappa////////////////////////////
//  static QCPCurve * curve_14 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
//  curve_14->data().reset();
//  curve_14->setPen(QPen(Qt::blue));
//  curve_14->setData(temp_x13,temp_y14);
//  curve_14->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
//                             QPen(Qt::black, 3), QBrush(Qt::white), 5 ) );
  ///////////////////////////////vehicle state kappa end////////////////////////////


  QPen graphPen;
  graphPen.setColor(QColor(0,0,0));
  graphPen.setWidthF(3);                                
  static QCPCurve * curve_15 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_15->data().reset();
  curve_15->setPen(graphPen);
  curve_15->setData(qoverlap_start_x,qoverlap_start_y);
  curve_15->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                             QPen(Qt::red, 5), QBrush(Qt::white), 10 ) );

  static QCPCurve * curve_16 = new QCPCurve(CustomPlot->xAxis, CustomPlot->yAxis);
  curve_16->data().reset();
  curve_16->setPen(graphPen);
  curve_16->setData(qoverlap_end_x,qoverlap_end_y);
  curve_16->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDot,
                             QPen(Qt::red, 5), QBrush(Qt::white), 10 ) );


 //////////////////////////Pnc status visualization //////////////////////////////


   ui->ModeValue->setText(QString::fromStdString(PncInformation::Instance()->GetModeString()));
   ui->LocationStatusValue->setText(QString::fromStdString(PncInformation::Instance()->GetLocationStatusString()));
   ui->ProjectionStatusValue->setText(QString::fromStdString(PncInformation::Instance()->GetProjectionStatusString()));
   ui->WidthStatusValue->setText(QString::fromStdString(PncInformation::Instance()->GetWidthStatusString()));
   ui->RouteStatusValue->setText(QString::fromStdString(PncInformation::Instance()->GetRouteStatusString()));
   ui->IntersectionStatusValue->setText( QString::fromStdString(PncInformation::Instance()->GetIntersectionStatusString()));
   ui->SwitchStatusValue->setText(QString::fromStdString(PncInformation::Instance()->GetSwitchStatusString()));

   QFont status_font;
   QPalette  status_palette;
   status_font.setBold(QFont::Bold);
   auto current_final_status=PncInformation::Instance()->GetPncStatus().final_status;
   if(current_final_status==PncInformation::FinalStatus::FINAL_OK){
       status_palette.setColor(QPalette::Text,Qt::green);
   }
   else if(current_final_status==PncInformation::FinalStatus::FINAL_ERROR){
       status_palette.setColor(QPalette::Text,Qt::red);
   }
   ui->FinalStatusValue->setText( QString::fromStdString(PncInformation::Instance()->GetFinalStatusString()));
   ui->FinalStatusValue->setFont(status_font);
   ui->FinalStatusValue->setPalette(status_palette);

   ui->CurrentSceneIdValue->setText(QString::number(PncInformation::Instance()->GetPncStatus().scene_id,'f',0));
   ui->CurrentMapIdValue->setText(QString::number(PncInformation::Instance()->GetPncStatus().map_id,'f',0));
   ui->CurrentRoutesNumValue->setText( QString::number(PncInformation::Instance()->GetPncStatus().routes_number));

   ui->CurrentRoutesInfoValue->setText("");
   auto temp_text_context=ui->CurrentRoutesInfoValue->toPlainText();
   for(auto route_info:PncInformation::Instance()->GetPncStatus().routes_info){
       string lane_ids_string="";
       for(auto lane_id:route_info.lanes_ids){
           lane_ids_string=lane_ids_string+to_string(lane_id)+", ";
       }
       //////////side slip to string
       string current_side_slip;
       if(route_info.side_slip==true){current_side_slip="true";}
       else{current_side_slip="false";}

       /////////////circular textcontent
//       ui->CurrentRoutesInfoValue->setText("");
       ui->CurrentRoutesInfoValue->setText( "route id: "+QString::number(route_info.route_id,'f',0)+"\n"+
                                                                                   "side slip: "+QString::fromStdString(current_side_slip)+"\n"+
                                                                                   "lane ids: "+QString::fromStdString(lane_ids_string)+"\n"+
                                                                                   "points number:"+QString::number(route_info.points_number,'f',0)+"\n"+"\n"+
                                                                                     temp_text_context );
        temp_text_context=ui->CurrentRoutesInfoValue->toPlainText();


   }

 /////////////////////////Pnc status visualization end////////////////////////////////


//  ui->PlanningTime->setText(QString::number(t2,'f',1) +" ms");
  CustomPlot->xAxis->setLabel(
//                               " t1 = " + QString::number(t1,'f',0)+
//                               " t3 = " + QString::number(t3,'f',1)+
                               " run_times = " + QString::number(run_times,'f',1)
                              );
  CustomPlot->yAxis->setLabel("mV");
  CustomPlot->setInteractions(QCP::iRangeDrag|QCP::iRangeZoom| QCP::iSelectAxes |
                              QCP::iSelectLegend | QCP::iSelectPlottables);
  CustomPlot->axisRect()->setupFullAxesBox();
  CustomPlot->replot();
  CustomPlot->clearGraphs();
//  CustomPlot->clearItems();
//  CustomPlot->clearMask();
//  CustomPlot->clearPlottables();


#endif
  cout<<std_planning.Name()<<endl;
  run_times++;
  this->update();

 }


prediction::PredictionObstacle MainWindow::CreatObstacle(int id,
      double x,     double y,    double theta, double v_x,double v_y,
      double length,double width,double height,int polygon_num,
      int tra_num,VehicleState vehicle_state ) {

  prediction::PredictionObstacle  obstacle1;
  obstacle1.perception_obstacle.id = id;
  obstacle1.perception_obstacle.position.x = x;
  obstacle1.perception_obstacle.position.y = y;
  obstacle1.perception_obstacle.theta      = theta ;//in world coordinate heading
  obstacle1.perception_obstacle.velocity.y = v_y;
  obstacle1.perception_obstacle.velocity.x = v_x;
  obstacle1.perception_obstacle.length = length;
  obstacle1.perception_obstacle.width  =  width;
  obstacle1.perception_obstacle.height = height;
  obstacle1.perception_obstacle.polygon_num = polygon_num;
  obstacle1.trajectory_num = tra_num;
  //rotate angle in world coordinate
  for(int i = 0;i<polygon_num;i++ )
   {
      perception::Point point1 = obstacle1.perception_obstacle.position ;
      perception::Point point = {0,0,0} ;

      if(i<2){
         point.x += obstacle1.perception_obstacle.length/2.0;
         if(i%2 == 0)
          point.y += obstacle1.perception_obstacle.width/2.0;
         else
          point.y -= obstacle1.perception_obstacle.width/2.0;
       }else{
          point.x -= obstacle1.perception_obstacle.length/2.0;
          if(i%2 == 0)
           point.y -= obstacle1.perception_obstacle.width/2.0;
          else
           point.y += obstacle1.perception_obstacle.width/2.0;
       }
      CoordinateConvert::TranslationAndRotaion( point, point1,
                                                -obstacle1.perception_obstacle.theta);
      obstacle1.perception_obstacle.polygon_point.push_back(point);

   }

  perception::Point point1 = {0,0,0} ;

  for(int i = 0;i<obstacle1.perception_obstacle.polygon_point.size();i++ )
   {
      perception::Point point =
          obstacle1.perception_obstacle.polygon_point.at(i) ;
      point.x -= vehicle_state.x;
      point.y -= vehicle_state.y;
      CoordinateConvert::TranslationAndRotaion( point, point1,
                                                vehicle_state.heading);
      obstacle1.perception_obstacle.polygon_point.at(i) =  point;
   }
  //
  //theta ( -M_PI ~ M_PI )
  double temp_theta = obstacle1.perception_obstacle.theta ;
  temp_theta -= M_PI/2.0;//heading convert yaw
  if(temp_theta > M_PI){
      temp_theta -= 2.0*M_PI;
   } else if(temp_theta < -M_PI){
      temp_theta += 2.0*M_PI;
   }
  temp_theta -= vehicle_state.heading;//heading convert car coordinate y aixs
    if(temp_theta > M_PI){
        temp_theta -= 2.0*M_PI;
     } else if(temp_theta < -M_PI){
        temp_theta += 2.0*M_PI;
     }
    //convert car x aixs 0~2PI
    temp_theta += M_PI_2;
    if(temp_theta < 0){
        temp_theta += 2.0*M_PI;
     }

  obstacle1.perception_obstacle.theta  = temp_theta;
  obstacle1.perception_obstacle.position.x -= vehicle_state.x;
  obstacle1.perception_obstacle.position.y -= vehicle_state.y;
  CoordinateConvert::TranslationAndRotaion( obstacle1.perception_obstacle.position,
                                            point1,
                                            vehicle_state.heading);
  if(tra_num == 1) {
  prediction::Trajectory tra_;
  TrajectoryPoint point;
  for(int i = 0 ; i < 80 ; i++){
      point.relative_time = 0.1 * i;
      point.v = sqrt(v_x*v_x + v_y*v_y);
      point.a = 0;
      point.path_point.x = x - vehicle_state.x + point.relative_time*v_x;
      point.path_point.y = y - vehicle_state.y + point.relative_time*v_y;
      CoordinateConvert::TranslationAndRotaion( point.path_point,
                                                point1,
                                                vehicle_state.heading);
      point.path_point.theta = temp_theta;
      //point.path_point.s = 0.5 *i;
      tra_.trajectory_point.push_back(point);
    }
  tra_.probability = 0.8;
  tra_.trajectory_point_num = 80;
  obstacle1.trajectory.push_back(tra_);
 }
  obstacle1.perception_obstacle.theta =
      obstacle1.perception_obstacle.theta /M_PI * 180.0;
 return obstacle1;

}

prediction::PredictionObstacle MainWindow::CreatObstacle(int id,
  double x,     double y,    double theta, double v_x,     double v_y,
  double length,double width,double height,int polygon_num,int tra_num){

  prediction::PredictionObstacle  obstacle1;
  obstacle1.perception_obstacle.id = id;
  obstacle1.perception_obstacle.position.x = x;
  obstacle1.perception_obstacle.position.y = y;
  obstacle1.perception_obstacle.theta      = theta ;
  obstacle1.perception_obstacle.velocity.y  = v_y;
  obstacle1.perception_obstacle.velocity.x  = v_x;
  obstacle1.perception_obstacle.length = length;
  obstacle1.perception_obstacle.width  =  width;
  obstacle1.perception_obstacle.height = height;
  obstacle1.perception_obstacle.polygon_num = polygon_num;
  obstacle1.trajectory_num = tra_num;
  //obstacle1.perception_obstacle.theta  -= M_PI/2.0; //convert to yaw
  for(int i = 0;i<obstacle1.perception_obstacle.polygon_num;i++ )
   {
      perception::Point point1 = obstacle1.perception_obstacle.position ;
      perception::Point point = {0,0,0} ;
      if(i<2){
          point.x += obstacle1.perception_obstacle.length/2.0;
         if(i%2 == 0)
          point.y += obstacle1.perception_obstacle.width/2.0;
         else
          point.y -= obstacle1.perception_obstacle.width/2.0;
       }else{
          point.x -= obstacle1.perception_obstacle.length/2.0;
          if(i%2 == 0)
           point.y -= obstacle1.perception_obstacle.width/2.0;
          else
           point.y += obstacle1.perception_obstacle.width/2.0;
       }

      CoordinateConvert::TranslationAndRotaion( point, point1,
                                                -(obstacle1.perception_obstacle.theta -M_PI_2) );

      obstacle1.perception_obstacle.polygon_point.push_back(point);

   }

  if(obstacle1.trajectory_num == 1){
      prediction::Trajectory tra_;
      TrajectoryPoint point;
      for(int i = 0 ; i < 80 ; i++){
          point.relative_time = 0.1 * i;
          point.v = sqrt(25.25);
          point.a = 0;
          point.path_point.x = obstacle1.perception_obstacle.position.x - i*0.1*0.85;
          point.path_point.y = obstacle1.perception_obstacle.position.y + i*0.1*5;
          point.path_point.theta = i * 0.25;
          point.path_point.s = 0.5 *i;
          tra_.trajectory_point.push_back(point);
        }
      tra_.probability = 0.8;
      tra_.trajectory_point_num = 80;
      obstacle1.trajectory.push_back(tra_);
    }
  obstacle1.perception_obstacle.theta =
      obstacle1.perception_obstacle.theta /M_PI * 180.0;
  return obstacle1;

}

std::pair<vector<PathPoint>,vector<PathPoint>>
MainWindow::SetBoundaryLanePoints(std::vector<hdmap::MapPoint>reference_points,ReferenceLine* reference_line){

    std::vector<PathPoint> left_boundary_lane_points_;
    std::vector<PathPoint> right_boundary_lane_points_;

    for(auto temp_ref_point: reference_points){

        PathPoint temp_left_boundary_point;
        PathPoint temp_right_boundary_point;

        double left_lane_width=0.0;
        double right_lane_width=0.0;
        reference_line->GetLaneWidth(temp_ref_point.s,left_lane_width,right_lane_width);

        temp_left_boundary_point.s=temp_ref_point.s;
        temp_left_boundary_point.theta=temp_ref_point.euler_angles.yaw;
        temp_left_boundary_point.x=temp_ref_point.point_enu.x+left_lane_width*cos(M_PI+temp_left_boundary_point.theta);
        temp_left_boundary_point.y=temp_ref_point.point_enu.y+left_lane_width*sin(M_PI+temp_left_boundary_point.theta);
        temp_left_boundary_point.z=temp_ref_point.point_enu.z;

        temp_right_boundary_point.s=temp_ref_point.s;
        temp_right_boundary_point.theta=temp_ref_point.euler_angles.yaw;
        temp_right_boundary_point.x=temp_ref_point.point_enu.x+right_lane_width*cos(temp_left_boundary_point.theta);
        temp_right_boundary_point.y=temp_ref_point.point_enu.y+right_lane_width*sin(temp_left_boundary_point.theta);
        temp_right_boundary_point.z=temp_ref_point.point_enu.z;


        left_boundary_lane_points_.push_back(temp_left_boundary_point);
        right_boundary_lane_points_.push_back(temp_right_boundary_point);
    }
    std::pair<vector<PathPoint>,vector<PathPoint>> boundary_lane_points_pair(make_pair(left_boundary_lane_points_,
                                                                                                                                            right_boundary_lane_points_));
    return boundary_lane_points_pair;
}

const hdmap::MapPoint MainWindow::GetLaneMatchPoint(const hdmap::Lane& lane,double s) const{

  if(s < lane.central_points.front().s ){
      return lane.central_points.front();
   }else if(s > lane.central_points.back().s){
      return lane.central_points.back();
   }

  for( int iter = 0 ;iter < lane.central_points.size();iter++ )
  {
    //search for nearset point
    if( lane.central_points.at(iter).s >= s )
     {
       if(iter - 1 >= 0 )
       {
         return abs(lane.central_points.at(iter).s - s) <
                abs(lane.central_points.at(iter - 1).s - s)?
                lane.central_points.at(iter):
                lane.central_points.at(iter - 1);
        }else{
          return lane.central_points.at(iter);
        }
      }
  }

}

void  MainWindow::GetCarCoordinateCleanData( CleanTarget  &clean_target,
                                       const VehicleState &vehicle_state){

  for(auto & garbage : clean_target.Garbages){

      Point3D point1 = {0,0,0} ;
      garbage.position.x -= vehicle_state.x;
      garbage.position.y -= vehicle_state.y;
      CoordinateConvert::TranslationAndRotaion( garbage.position, point1,
                                                vehicle_state.heading );
   }

}


QCPCurve * MainWindow::NewCurve(QCustomPlot *customPlot) {

    return new QCPCurve(customPlot->xAxis,customPlot->yAxis);
}

void MainWindow::NewCurves(QCustomPlot *customPlot){

     static QCPCurve *
     curve01 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve01);

     static QCPCurve *
     curve02 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve02);

     static QCPCurve *
     curve03 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve03);

     static QCPCurve *
     curve04 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve04);

     static QCPCurve *
     curve05 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve05);

     static QCPCurve *
     curve06 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve06);

     static QCPCurve *
     curve07 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve07);

     static QCPCurve *
     curve08 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve08);

     static QCPCurve *
     curve09 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve09);

     static QCPCurve *
     curve10 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve10);

     static QCPCurve *
     curve11 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve11);

     static QCPCurve *
     curve12 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve12);

     static QCPCurve *
     curve13 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve13);

     static QCPCurve *
     curve14 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve14);

     static QCPCurve *
     curve15 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve15);

     static QCPCurve *
     curve16 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve16);

     static QCPCurve *
     curve17 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve17);

     static QCPCurve *
     curve18 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve18);

     static QCPCurve *
     curve19 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve19);

     static QCPCurve *
     curve20 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve20);

     static QCPCurve *
     curve21 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve21);

     static QCPCurve *
     curve22 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve22);

     static QCPCurve *
     curve23 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve23);

     static QCPCurve *
     curve24 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve24);

     static QCPCurve *
     curve25 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve25);

     static QCPCurve *
     curve26 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve26);

     static QCPCurve *
     curve27 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve27);

     static QCPCurve *
     curve28 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve28);

     static QCPCurve *
     curve29 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve29);

     static QCPCurve *
     curve30 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve30);

     static QCPCurve *
     curve31 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve31);

     static QCPCurve *
     curve32 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve32);

     static QCPCurve *
     curve33 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve33);

     static QCPCurve *
     curve34 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve34);

     static QCPCurve *
     curve35 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve35);

     static QCPCurve *
     curve36 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve36);

     static QCPCurve *
     curve37 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve37);

     static QCPCurve *
     curve38 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve38);

     static QCPCurve *
     curve39 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve39);

     static QCPCurve *
     curve40 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve40);

     static QCPCurve *
     curve41 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve41);

     static QCPCurve *
     curve42 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve42);

     static QCPCurve *
     curve43 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve43);

     static QCPCurve *
     curve44 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve44);

     static QCPCurve *
     curve45 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve45);

     static QCPCurve *
     curve46 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve46);

     static QCPCurve *
     curve47 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve47);

     static QCPCurve *
     curve48 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve48);

     static QCPCurve *
     curve49 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve49);

     static QCPCurve *
     curve50 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve50);

     static QCPCurve *
     curve51 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve51);

     static QCPCurve *
     curve52 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve52);

     static QCPCurve *
     curve53 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve53);

     static QCPCurve *
     curve54 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve54);

     static QCPCurve *
     curve55 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve55);

     static QCPCurve *
     curve56 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve56);

     static QCPCurve *
     curve57 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve57);

     static QCPCurve *
     curve58 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve58);

     static QCPCurve *
     curve59 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve59);

     static QCPCurve *
     curve60 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve60);

     static QCPCurve *
     curve61 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve61);

     static QCPCurve *
     curve62 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve62);

     static QCPCurve *
     curve63 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve63);

     static QCPCurve *
     curve64 = NewCurve(customPlot);
     qcp_ploygon_ptrs_.push_back(curve64);
 }

void MainWindow::NewRects(QCustomPlot *customPlot){

    static QCPCurve *
    curve01 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve01);

    static QCPCurve *
    curve02 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve02);

    static QCPCurve *
    curve03 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve03);

    static QCPCurve *
    curve04 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve04);

    static QCPCurve *
    curve05 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve05);

    static QCPCurve *
    curve06 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve06);

    static QCPCurve *
    curve07 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve07);

    static QCPCurve *
    curve08 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve08);

    static QCPCurve *
    curve09 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve09);

    static QCPCurve *
    curve10 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve10);

    static QCPCurve *
    curve11 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve11);

    static QCPCurve *
    curve12 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve12);

    static QCPCurve *
    curve13 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve13);

    static QCPCurve *
    curve14 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve14);

    static QCPCurve *
    curve15 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve15);

    static QCPCurve *
    curve16 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve16);

    static QCPCurve *
    curve17 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve17);

    static QCPCurve *
    curve18 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve18);

    static QCPCurve *
    curve19 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve19);

    static QCPCurve *
    curve20 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve20);

    static QCPCurve *
    curve21 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve21);

    static QCPCurve *
    curve22 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve22);

    static QCPCurve *
    curve23 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve23);

    static QCPCurve *
    curve24 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve24);

    static QCPCurve *
    curve25 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve25);

    static QCPCurve *
    curve26 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve26);

    static QCPCurve *
    curve27 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve27);

    static QCPCurve *
    curve28 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve28);

    static QCPCurve *
    curve29 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve29);

    static QCPCurve *
    curve30 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve30);

    static QCPCurve *
    curve31 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve31);

    static QCPCurve *
    curve32 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve32);

    static QCPCurve *
    curve33 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve33);

    static QCPCurve *
    curve34 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve34);

    static QCPCurve *
    curve35 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve35);

    static QCPCurve *
    curve36 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve36);

    static QCPCurve *
    curve37 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve37);

    static QCPCurve *
    curve38 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve38);

    static QCPCurve *
    curve39 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve39);

    static QCPCurve *
    curve40 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve40);

    static QCPCurve *
    curve41 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve41);

    static QCPCurve *
    curve42 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve42);

    static QCPCurve *
    curve43 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve43);

    static QCPCurve *
    curve44 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve44);

    static QCPCurve *
    curve45 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve45);

    static QCPCurve *
    curve46 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve46);

    static QCPCurve *
    curve47 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve47);

    static QCPCurve *
    curve48 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve48);

    static QCPCurve *
    curve49 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve49);

    static QCPCurve *
    curve50 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve50);

    static QCPCurve *
    curve51 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve51);

    static QCPCurve *
    curve52 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve52);

    static QCPCurve *
    curve53 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve53);

    static QCPCurve *
    curve54 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve54);

    static QCPCurve *
    curve55 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve55);

    static QCPCurve *
    curve56 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve56);

    static QCPCurve *
    curve57 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve57);

    static QCPCurve *
    curve58 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve58);

    static QCPCurve *
    curve59 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve59);

    static QCPCurve *
    curve60 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve60);

    static QCPCurve *
    curve61 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve61);

    static QCPCurve *
    curve62 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve62);

    static QCPCurve *
    curve63 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve63);

    static QCPCurve *
    curve64 = NewCurve(customPlot);
    qcp_rect_ptrs_.push_back(curve64);
}

void MainWindow::NewTras(QCustomPlot *customPlot){

    static QCPCurve *
    curve01 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve01);

    static QCPCurve *
    curve02 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve02);

    static QCPCurve *
    curve03 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve03);

    static QCPCurve *
    curve04 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve04);

    static QCPCurve *
    curve05 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve05);

    static QCPCurve *
    curve06 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve06);

    static QCPCurve *
    curve07 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve07);

    static QCPCurve *
    curve08 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve08);

    static QCPCurve *
    curve09 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve09);

    static QCPCurve *
    curve10 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve10);

    static QCPCurve *
    curve11 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve11);

    static QCPCurve *
    curve12 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve12);

    static QCPCurve *
    curve13 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve13);

    static QCPCurve *
    curve14 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve14);

    static QCPCurve *
    curve15 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve15);

    static QCPCurve *
    curve16 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve16);

    static QCPCurve *
    curve17 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve17);

    static QCPCurve *
    curve18 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve18);

    static QCPCurve *
    curve19 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve19);

    static QCPCurve *
    curve20 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve20);

    static QCPCurve *
    curve21 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve21);

    static QCPCurve *
    curve22 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve22);

    static QCPCurve *
    curve23 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve23);

    static QCPCurve *
    curve24 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve24);

    static QCPCurve *
    curve25 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve25);

    static QCPCurve *
    curve26 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve26);

    static QCPCurve *
    curve27 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve27);

    static QCPCurve *
    curve28 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve28);

    static QCPCurve *
    curve29 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve29);

    static QCPCurve *
    curve30 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve30);

    static QCPCurve *
    curve31 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve31);

    static QCPCurve *
    curve32 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve32);

    static QCPCurve *
    curve33 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve33);

    static QCPCurve *
    curve34 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve34);

    static QCPCurve *
    curve35 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve35);

    static QCPCurve *
    curve36 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve36);

    static QCPCurve *
    curve37 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve37);

    static QCPCurve *
    curve38 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve38);

    static QCPCurve *
    curve39 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve39);

    static QCPCurve *
    curve40 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve40);

    static QCPCurve *
    curve41 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve41);

    static QCPCurve *
    curve42 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve42);

    static QCPCurve *
    curve43 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve43);

    static QCPCurve *
    curve44 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve44);

    static QCPCurve *
    curve45 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve45);

    static QCPCurve *
    curve46 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve46);

    static QCPCurve *
    curve47 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve47);

    static QCPCurve *
    curve48 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve48);

    static QCPCurve *
    curve49 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve49);

    static QCPCurve *
    curve50 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve50);

    static QCPCurve *
    curve51 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve51);

    static QCPCurve *
    curve52 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve52);

    static QCPCurve *
    curve53 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve53);

    static QCPCurve *
    curve54 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve54);

    static QCPCurve *
    curve55 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve55);

    static QCPCurve *
    curve56 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve56);

    static QCPCurve *
    curve57 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve57);

    static QCPCurve *
    curve58 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve58);

    static QCPCurve *
    curve59 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve59);

    static QCPCurve *
    curve60 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve60);

    static QCPCurve *
    curve61 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve61);

    static QCPCurve *
    curve62 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve62);

    static QCPCurve *
    curve63 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve63);

    static QCPCurve *
    curve64 = NewCurve(customPlot);
    qcp_tra_ptrs_.push_back(curve64);
}

void MainWindow::NewIds(QCustomPlot *customPlot){

    static QCPItemText *
    textLabel01 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel01);

    static QCPItemText *
    textLabel02 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel02);

    static QCPItemText *
    textLabel03 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel03);

    static QCPItemText *
    textLabel04 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel04);

    static QCPItemText *
    textLabel05 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel05);

    static QCPItemText *
    textLabel06 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel06);

    static QCPItemText *
    textLabel07 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel07);

    static QCPItemText *
    textLabel08 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel08);

    static QCPItemText *
    textLabel09 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel09);

    static QCPItemText *
    textLabel10 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel10);

    static QCPItemText *
    textLabel11 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel11);

    static QCPItemText *
    textLabel12 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel12);

    static QCPItemText *
    textLabel13 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel13);

    static QCPItemText *
    textLabel14 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel14);

    static QCPItemText *
    textLabel15 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel15);

    static QCPItemText *
    textLabel16 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel16);

    static QCPItemText *
    textLabel17 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel17);

    static QCPItemText *
    textLabel18 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel18);

    static QCPItemText *
    textLabel19 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel19);

    static QCPItemText *
    textLabel20 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel20);

    static QCPItemText *
    textLabel21 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel21);

    static QCPItemText *
    textLabel22 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel22);

    static QCPItemText *
    textLabel23 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel23);

    static QCPItemText *
    textLabel24 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel24);

    static QCPItemText *
    textLabel25 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel25);

    static QCPItemText *
    textLabel26 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel26);

    static QCPItemText *
    textLabel27 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel27);

    static QCPItemText *
    textLabel28 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel28);

    static QCPItemText *
    textLabel29 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel29);

    static QCPItemText *
    textLabel30 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel30);

    static QCPItemText *
    textLabel31 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel31);

    static QCPItemText *
    textLabel32 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel32);

    static QCPItemText *
    textLabel33 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel33);

    static QCPItemText *
    textLabel34 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel34);

    static QCPItemText *
    textLabel35 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel35);

    static QCPItemText *
    textLabel36 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel36);

    static QCPItemText *
    textLabel37 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel37);

    static QCPItemText *
    textLabel38 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel38);

    static QCPItemText *
    textLabel39 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel39);

    static QCPItemText *
    textLabel40 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel40);

    static QCPItemText *
    textLabel41 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel41);

    static QCPItemText *
    textLabel42 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel42);

    static QCPItemText *
    textLabel43 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel43);

    static QCPItemText *
    textLabel44 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel44);

    static QCPItemText *
    textLabel45 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel45);

    static QCPItemText *
    textLabel46 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel46);

    static QCPItemText *
    textLabel47 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel47);

    static QCPItemText *
    textLabel48 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel48);

    static QCPItemText *
    textLabel49 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel49);

    static QCPItemText *
    textLabel50 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel50);

    static QCPItemText *
    textLabel51 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel51);

    static QCPItemText *
    textLabel52 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel52);

    static QCPItemText *
    textLabel53 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel53);

    static QCPItemText *
    textLabel54 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel54);

    static QCPItemText *
    textLabel55 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel55);

    static QCPItemText *
    textLabel56 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel56);

    static QCPItemText *
    textLabel57 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel57);

    static QCPItemText *
    textLabel58 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel58);

    static QCPItemText *
    textLabel59 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel59);

    static QCPItemText *
    textLabel60 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel60);

    static QCPItemText *
    textLabel61 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel61);

    static QCPItemText *
    textLabel62 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel62);

    static QCPItemText *
    textLabel63 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel63);

    static QCPItemText *
    textLabel64 = new QCPItemText(customPlot);
    qcp_id_ptrs_.push_back(textLabel64);
}

void MainWindow::NewHeadings(QCustomPlot *customPlot){

    static QCPItemLine *
    textLabel01 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel01);

    static QCPItemLine *
    textLabel02 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel02);

    static QCPItemLine *
    textLabel03 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel03);

    static QCPItemLine *
    textLabel04 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel04);

    static QCPItemLine *
    textLabel05 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel05);

    static QCPItemLine *
    textLabel06 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel06);

    static QCPItemLine *
    textLabel07 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel07);

    static QCPItemLine *
    textLabel08 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel08);

    static QCPItemLine *
    textLabel09 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel09);

    static QCPItemLine *
    textLabel10 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel10);

    static QCPItemLine *
    textLabel11 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel11);

    static QCPItemLine *
    textLabel12 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel12);

    static QCPItemLine *
    textLabel13 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel13);

    static QCPItemLine *
    textLabel14 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel14);

    static QCPItemLine *
    textLabel15 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel15);

    static QCPItemLine *
    textLabel16 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel16);

    static QCPItemLine *
    textLabel17 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel17);

    static QCPItemLine *
    textLabel18 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel18);

    static QCPItemLine *
    textLabel19 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel19);

    static QCPItemLine *
    textLabel20 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel20);

    static QCPItemLine *
    textLabel21 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel21);

    static QCPItemLine *
    textLabel22 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel22);

    static QCPItemLine *
    textLabel23 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel23);

    static QCPItemLine *
    textLabel24 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel24);

    static QCPItemLine *
    textLabel25 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel25);

    static QCPItemLine *
    textLabel26 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel26);

    static QCPItemLine *
    textLabel27 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel27);

    static QCPItemLine *
    textLabel28 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel28);

    static QCPItemLine *
    textLabel29 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel29);

    static QCPItemLine *
    textLabel30 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel30);

    static QCPItemLine *
    textLabel31 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel31);

    static QCPItemLine *
    textLabel32 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel32);

    static QCPItemLine *
    textLabel33 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel33);

    static QCPItemLine *
    textLabel34 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel34);

    static QCPItemLine *
    textLabel35 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel35);

    static QCPItemLine *
    textLabel36 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel36);

    static QCPItemLine *
    textLabel37 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel37);

    static QCPItemLine *
    textLabel38 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel38);

    static QCPItemLine *
    textLabel39 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel39);

    static QCPItemLine *
    textLabel40 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel40);

    static QCPItemLine *
    textLabel41 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel41);

    static QCPItemLine *
    textLabel42 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel42);

    static QCPItemLine *
    textLabel43 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel43);

    static QCPItemLine *
    textLabel44 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel44);

    static QCPItemLine *
    textLabel45 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel45);

    static QCPItemLine *
    textLabel46 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel46);

    static QCPItemLine *
    textLabel47 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel47);

    static QCPItemLine *
    textLabel48 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel48);

    static QCPItemLine *
    textLabel49 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel49);

    static QCPItemLine *
    textLabel50 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel50);

    static QCPItemLine *
    textLabel51 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel51);

    static QCPItemLine *
    textLabel52 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel52);

    static QCPItemLine *
    textLabel53 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel53);

    static QCPItemLine *
    textLabel54 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel54);

    static QCPItemLine *
    textLabel55 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel55);

    static QCPItemLine *
    textLabel56 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel56);

    static QCPItemLine *
    textLabel57 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel57);

    static QCPItemLine *
    textLabel58 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel58);

    static QCPItemLine *
    textLabel59 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel59);

    static QCPItemLine *
    textLabel60 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel60);

    static QCPItemLine *
    textLabel61 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel61);

    static QCPItemLine *
    textLabel62 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel62);

    static QCPItemLine *
    textLabel63 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel63);

    static QCPItemLine *
    textLabel64 = new QCPItemLine(customPlot);
    qcp_heading_ptrs_.push_back(textLabel64);
}


//void MainWindow::on_CleanMode_clicked()
//{

//  clean_mode++;

//}

//void MainWindow::on_ParkingMode_clicked()
//{

//  park_mode++;


//}

void MainWindow::NewLeftBoundary(QCustomPlot* customPlot){
    static QCPCurve* curve1=NewCurve(customPlot);
    qcp_left_lane_boudary_ptrs_.push_back(curve1);

    static QCPCurve* curve2=NewCurve(customPlot);
    qcp_left_lane_boudary_ptrs_.push_back(curve2);

    static QCPCurve* curve3=NewCurve(customPlot);
    qcp_left_lane_boudary_ptrs_.push_back(curve3);

}

void MainWindow::NewRightBoundary(QCustomPlot* customPlot){
    static QCPCurve* curve1=NewCurve(customPlot);
    qcp_right_lane_boudary_ptrs_.push_back(curve1);

    static QCPCurve* curve2=NewCurve(customPlot);
    qcp_right_lane_boudary_ptrs_.push_back(curve2);

    static QCPCurve* curve3=NewCurve(customPlot);
    qcp_right_lane_boudary_ptrs_.push_back(curve3);

}

void MainWindow::NewStations(QCustomPlot* customPlot){
    static QCPCurve* curve1=NewCurve(customPlot);
    qcp_stations_ptrs_.push_back(curve1);

    static QCPCurve* curve2=NewCurve(customPlot);
    qcp_stations_ptrs_.push_back(curve2);

    static QCPCurve* curve3=NewCurve(customPlot);
    qcp_stations_ptrs_.push_back(curve3);

    static QCPCurve* curve4=NewCurve(customPlot);
    qcp_stations_ptrs_.push_back(curve4);

    static QCPCurve* curve5=NewCurve(customPlot);
    qcp_stations_ptrs_.push_back(curve5);

    static QCPCurve* curve6=NewCurve(customPlot);
    qcp_stations_ptrs_.push_back(curve6);

}

void MainWindow::NewTerminals(QCustomPlot* customPlot){
    static QCPCurve* curve1=NewCurve(customPlot);
    qcp_terminals_ptrs_.push_back(curve1);

    static QCPCurve* curve2=NewCurve(customPlot);
    qcp_terminals_ptrs_.push_back(curve2);

    static QCPCurve* curve3=NewCurve(customPlot);
    qcp_terminals_ptrs_.push_back(curve3);

    static QCPCurve* curve4=NewCurve(customPlot);
    qcp_terminals_ptrs_.push_back(curve4);

    static QCPCurve* curve5=NewCurve(customPlot);
    qcp_terminals_ptrs_.push_back(curve5);

    static QCPCurve* curve6=NewCurve(customPlot);
    qcp_terminals_ptrs_.push_back(curve6);

}

void MainWindow::NewSites(QCustomPlot* customPlot){
    static QCPCurve* curve1=NewCurve(customPlot);
    qcp_sites_ptrs_.push_back(curve1);

    static QCPCurve* curve2=NewCurve(customPlot);
    qcp_sites_ptrs_.push_back(curve2);

    static QCPCurve* curve3=NewCurve(customPlot);
    qcp_sites_ptrs_.push_back(curve3);

    static QCPCurve* curve4=NewCurve(customPlot);
    qcp_sites_ptrs_.push_back(curve4);

    static QCPCurve* curve5=NewCurve(customPlot);
    qcp_sites_ptrs_.push_back(curve5);

    static QCPCurve* curve6=NewCurve(customPlot);
    qcp_sites_ptrs_.push_back(curve6);

}










