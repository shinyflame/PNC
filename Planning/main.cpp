#include "mainwindow.h"
#include <QApplication>
#include "common/struct.h"
#include "config/planning_config_param_initial.h"
#include "common/speed/speed_data.h"
#include <stdio.h>        //printf()
#include <unistd.h>        //pause()
#include <signal.h>        //signal()
#include <string.h>        //memset()
#include <sys/time.h>    //struct itimerval, setitimer()
#include <iostream>

extern ConfigParam g_config_param;

#include <thread>         // std::thread
#include <mutex>          // std::mutex, std::unique_lock

std::mutex mtx;           // mutex for critical section

void print_block (int n, char c) {
    // critical section (exclusive access to std::cout signaled by lifetime of lck):
    std::unique_lock<std::mutex> lck (mtx);
    for (int i=0; i<n; ++i) {
        std::cout << c;
    }
    std::cout << '\n';
}

void printMes()
{
    static int count = 0;
    count++;
    std::cout<<"Get a SIGALRM: "<<count<<endl;
}


int main(int argc, char *argv[])
{
//  std::vector<TrajectoryPoint> trajectory_points_;
//  prediction::PredictionObstacles  prediction_obstacles;
  std::thread th1 (print_block,50,'*');
  std::thread th2 (print_block,50,'$');

  th1.join();
  th2.join();
QWidget* parent=0;
//  cout<<std::numeric_limits<double>::lowest()<<endl;
//  cout<<"std::numeric_limits<double>::infinity() = "
//      <<std::numeric_limits<double>::infinity()<<endl;
  QApplication a(argc, argv);
  MainWindow w1(parent);
  w1.show();
  return a.exec();
}
