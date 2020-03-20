#ifndef GET_NOW_TIME_H
#define GET_NOW_TIME_H

#include<sys/time.h>
#include <iostream>
#include <string>

//struct  timeval{
//
//       long  tv_sec;  /*秒*/
//       long  tv_usec; /*微妙*/
//}；

namespace Clock {
  double NowInHours();
  double NowInSeconds();

  unsigned long long NowInMs();
  std::string GetYMDHMS(bool is_show_second);
  std::string FromMsToYMDHMS(unsigned long long ms);



}//namespace Clock



#endif // GET_NOW_TIME_H
