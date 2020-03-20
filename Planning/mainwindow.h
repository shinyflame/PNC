#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qcustomplot.h>
#include "common/struct.h"
#include "std_planning.h"
#include "map/map_struct.h"
#include "common_struct.h"
#include "../map/hdmap.h"
#include "../map/common/pnc_information.h"

using namespace planning;

typedef string (*version)   ();
typedef bool   (*MapInit)   (string hdmap_path_,string config_path_);
typedef bool   (*MapRunOnce)(const MSFLoutput &msfl_output,
                             hdmap::Point3D &destination,
                             vector<int> &global_path,
                             hdmap::TrafficLightGroups &traffic_light_groups,
                             hdmap::PncRoutes &pnc_routes);

int & quote(int &a);
int normal(int a);

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent );
  ~MainWindow();
  void keyPressEvent(QKeyEvent *event);

  QPainter *painter ;
  QTimer *timer = new QTimer(this);
  void Graph_Show(QCustomPlot *customPlot);
  prediction::PredictionObstacle CreatObstacle(int id,
          double x, double y,double theta,double v_x,double v_y,
          double length,double width,double height,int polygon_num,int tra_num,
          VehicleState vehicle_state);
  prediction::PredictionObstacle CreatObstacle(int id,
          double x,     double y,    double theta, double v_x,     double v_y,
          double length,double width,double height,int polygon_num,int tra_num);


  void GetCarCoordinateCleanData( CleanTarget  &clean_target_,
                                  const VehicleState & vehicle_state);

  QCPCurve * NewCurve(QCustomPlot *customPlot);
  void NewCurves(QCustomPlot *customPlot);
  void NewRects(QCustomPlot *customPlot);
  void NewTras(QCustomPlot *customPlot);
  void NewIds(QCustomPlot *customPlot);
  void NewHeadings(QCustomPlot *customPlot);
  void NewLeftBoundary(QCustomPlot* customPlot);
  void NewRightBoundary(QCustomPlot* customPlot);
  void NewStations(QCustomPlot* customPlot);
  void NewTerminals(QCustomPlot* customPlot);
  void NewSites(QCustomPlot* customPlot);

  std::pair<vector<PathPoint>,vector<PathPoint>>
  SetBoundaryLanePoints(std::vector<hdmap::MapPoint>reference_points,ReferenceLine* reference_line);

 const hdmap::MapPoint GetLaneMatchPoint(const hdmap::Lane& lane,double s) const;
private slots:
  void on_pushButton_clicked();
  void Graph_Show();

  void on_auto_2_clicked();

  void on_Fast_Forward_clicked();

  void on_Slow_Forward_clicked();

private:
  Ui::MainWindow *ui;
  StdPlanning std_planning;
  hdmap::HdMap hdmap_interface;
  // routing::Routing routing_interface;

  int button = 0 ;
  int run_times = 0;
  int hdmap_runtimes_=0;
  bool first = true;
  bool auto_go =  false;
  SoftCommand command_;
  int clean_mode = 0;
  int park_mode = 0;
  hdmap::PncInformation::Mode config_mode_;
  MapRunOnce map_runonce;
  int skip_times=1;

  vector<QCPCurve *> qcp_ploygon_ptrs_;
  vector<QCPCurve *> qcp_rect_ptrs_;
  vector<QCPCurve *> qcp_tra_ptrs_;
  vector<QCPItemText *> qcp_id_ptrs_;
  vector<QCPItemLine *> qcp_heading_ptrs_;
  vector<QCPCurve*>  qcp_left_lane_boudary_ptrs_;
  vector<QCPCurve*>  qcp_right_lane_boudary_ptrs_;
  vector<QCPCurve*>  qcp_stations_ptrs_;
  vector<QCPCurve*>  qcp_terminals_ptrs_;
  vector<QCPCurve*>  qcp_sites_ptrs_;


};

#endif // MAINWINDOW_H
