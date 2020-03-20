
#include"get_now_time.h"
#include <cmath>
#include <iomanip>
#include <time.h>
#include <iostream>
using namespace std;
namespace Clock {

std::string GetYMDHMS(){

    struct timeval tStart;
    gettimeofday(&tStart, NULL);
    struct tm *info;
    info = gmtime(&tStart.tv_sec);
    int year = info->tm_year +1900;
    int mon  = info->tm_mon + 1;
    int day  = info->tm_mday;
    int hour = info->tm_hour+8;
    int min  = info->tm_min;
    int sec  = info->tm_sec;

    return std::to_string(year)+std::to_string(mon)+
           std::to_string(day)+std::to_string(hour)+
           std::to_string(min);//+std::to_string(sec);
}

double NowInHours(){

  struct timeval tStart;
  gettimeofday(&tStart, NULL);
  struct tm *info;
  info = gmtime(&tStart.tv_sec);

  cout<<"now hours = "<< info->tm_hour+8<<endl;

  return info->tm_hour+8;

  }
double NowInSeconds(){

  //struct timezone tz;
  struct timeval tStart;
  gettimeofday(&tStart, NULL);
  long double now_s = tStart.tv_sec;
  //long double now_ms= tStart.tv_usec/1000;
  long double now_time= now_s + tStart.tv_usec/1000000.0;
  //std::cout<<"now_time Second = "<<now_time<<std::endl;
  return now_time;

 }

unsigned long long NowInMs(){

  //struct timezone tz;
  struct timeval tStart;
  gettimeofday(&tStart, NULL);
  struct tm *info;
  info = gmtime(&tStart.tv_sec);
  unsigned long long now_time = tStart.tv_sec *1000 + tStart.tv_usec/1000;
  //printf("%d/%d/%d ", 1900 + info->tm_year, 1 + info->tm_mon, info->tm_mday);
  //printf("%d:%d:%d:%dms\n", info->tm_hour+8,info->tm_min, info->tm_sec,now_time%1000);

  //std::cout<<"now_time Ms = "<<now_time<<std::endl;
  //std::cout<<"now_time S  = "<<std::setprecision(13)<<now_time/1000.0<<std::endl;
  return now_time;

 }
}// namesapce Clock
