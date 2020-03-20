
#include <cmath>

#include "pnc_util.h"



namespace PncUtil {


int  GetMatchedIndexOnLane(const hdmap::Lane& lane,double s){

    int current_point_index=0;
    auto current_lane_point_num=lane.central_points_num;
    if(s<lane.central_points.front().s){
        current_point_index=0;
        }
    else if(s>lane.central_points.back().s){
        current_point_index=current_lane_point_num-1;
       }
    else{
           for(int point_index=0;point_index<current_lane_point_num;point_index++){

              if(s<=lane.central_points[point_index].s){
                  if(point_index==0){
                      current_point_index=point_index;
                       break;
                  }
                  else{
                      current_point_index=fabs(s-lane.central_points[point_index].s)>
                                                                 fabs(s-lane.central_points[point_index-1].s)?
                                                                (point_index-1):point_index;
                      break;
                  }
                }
             }
         }

     return current_point_index;
}
}
