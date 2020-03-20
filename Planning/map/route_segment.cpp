
#include "./route_segment.h"
#include "../math/math_utils.h"

namespace hdmap {
/// constructor brief
/// route is consitute from several lanes
/// lane_id is the vehicle current position corresponding lane's id
/// if the vehicle is on current route is_on_route is ture otherwise is false
RouteSegment::RouteSegment(Route route,int route_id, bool on_route,bool is_update,
           bool is_side_slip):route_(route),id_(route_id),is_on_segment_(on_route),
           update_state_(is_update),is_side_slip_(is_side_slip)

{
  double s;
  for(int i = 1; i < route_.reference_points.size(); i++ )
   {
      Vec2d start_point( route_.reference_points.at(i-1).point_enu.x,
                         route_.reference_points.at(i-1).point_enu.y );
      Vec2d end_point( route_.reference_points.at(i).point_enu.x,
                       route_.reference_points.at(i).point_enu.y );
      LineSegment2d line_segment(start_point,end_point);
      segments_.push_back(line_segment);
      if(accumulated_s_.empty())
       {
         s = line_segment.length();
         accumulated_s_.push_back(s);
       } else {

         s = line_segment.length() + accumulated_s_.back();
         accumulated_s_.push_back(s);
      }

   }
}

bool RouteSegment::GetProjection(const Vec2d &point, double &s, double&l) const
{
      if (route_.reference_points.empty()) {
        cout<<"Error reference points is empty !"<<endl;
        return false;
      }

      double min_dist = std::numeric_limits<double>::infinity();
      int points_num = route_.reference_points.size();
      int min_index = 0;
      for (int i = 0; i < points_num; ++i) {
        Vec2d reference_point(route_.reference_points.at(i).point_enu.x,
                              route_.reference_points.at(i).point_enu.y );
        const double distance = reference_point.DistanceSquareTo(point);
        if (distance < min_dist) {
          min_index = i;
          min_dist = distance;
        }
      }

      int last_index = min_index-1 > 0 ? min_index-1 : 0;
      int next_index = min_index+1 < points_num - 1 ? min_index + 1 : points_num - 1;
      const Vec2d start_point(route_.reference_points.at(last_index).point_enu.x,
                              route_.reference_points.at(last_index).point_enu.y);
      const Vec2d  next_point(route_.reference_points.at(next_index).point_enu.x,
                              route_.reference_points.at(next_index).point_enu.y);

      LineSegment2d nearest_seg(start_point,next_point);
      const auto prod = nearest_seg.ProductOntoUnit(point);//叉乘
      const auto proj = nearest_seg.ProjectOntoUnit(point);//点乘
      min_dist = std::sqrt(nearest_seg.DistanceSquareTo(point));//

      if (min_index == 0) {
        s = std::min(proj, nearest_seg.length());
        //叉乘的结果是正数，说明a到b是逆时针，反之顺时针；
        l = (prod > 0.0 ? 1 : -1) * min_dist;
      } else if (min_index == points_num - 1) {
        s = route_.reference_points[min_index].s + std::max(0.0, proj);
        l = (prod > 0.0 ? 1 : -1) * min_dist;
      } else {
        s = route_.reference_points[min_index].s +
            std::max(0.0, std::min(proj, nearest_seg.length()));
        l = (prod > 0.0 ? 1 : -1) * min_dist;
      }
      return true;
}

MapPoint RouteSegment::GetMatchPoint(double s) const{

  if(s < route_.reference_points.front().s ){
      return route_.reference_points.front();
   }else if(s > route_.reference_points.back().s){
      return route_.reference_points.back();
   }

  for( int iter = 0 ;iter < route_.reference_points.size();iter++ )
  {
    //search for nearset point
    if( route_.reference_points.at(iter).s >= s )
     {
       if(iter - 1 >= 0 )
       {
         return abs(route_.reference_points.at(iter).s - s) <
                abs(route_.reference_points.at(iter - 1).s - s)?
                route_.reference_points.at(iter):
                route_.reference_points.at(iter - 1);
        }else{
          return route_.reference_points.at(iter);
        }
      }
  }

}

bool RouteSegment::
GetLaneWidth(const double &s,double & left_width,double &right_width) const{

  if(s < route_.widths.front().s ){

      left_width  = route_.widths.front().center_to_left;
      right_width = route_.widths.front().center_to_right;
      return true;
   }else if(s > route_.widths.back().s){

      left_width  = route_.widths.back().center_to_left;
      right_width = route_.widths.back().center_to_right;
      return true;
   }

  for( int iter = 0 ;iter < route_.widths.size();iter++ ) {
    //search for nearset point
    if( route_.widths.at(iter).s >= s ){
      if(iter - 1 >= 0 )
      {
        if( abs(route_.widths.at(iter).s - s) <
            abs(route_.widths.at(iter - 1).s - s)) {

          left_width  = route_.widths.at(iter).center_to_left;
          right_width = route_.widths.at(iter).center_to_right;
        }else{

          left_width  = route_.widths.at(iter).center_to_left;
          right_width = route_.widths.at(iter).center_to_right;
        }
      } else {

        left_width  = route_.widths.at(iter).center_to_left;
        right_width = route_.widths.at(iter).center_to_right;
      }

      return true;  }
   }
  return false;
}
double RouteSegment::RouteSpeedLimitFromS(const double &s) const{

  if( s >= 0 && s <= route_.reference_points.back().s ){
    return route_.speed_limit;
    }
}


bool RouteSegment::OverlapWith(const Box2d &box, double width) const{

  const Vec2d center = box.center();
    const double radius_sqr = Sqr(box.diagonal() / 2.0 + width) + kMathEpsilon;
    for (const auto& segment : segments_) {
      if (segment.DistanceSquareTo(center) > radius_sqr) {
        continue;
      }
      if (box.DistanceTo(segment) <= width + kMathEpsilon) {
        return true;
      }
    }
    return false;
}
// from start_s to end_s search and compute point correspond  s_l
bool RouteSegment::
GetProjectionWithHeuristicParams(const Vec2d  &point,
                                 const double &start_s,
                                 const double &end_s,
                                       double &s,
                                       double &l,
                                       double &min_distance) const
{
  if (segments_.empty()) { return false; }

    int  num_segments_ = segments_.size();
    min_distance = std::numeric_limits<double>::infinity();

    int start_interpolation_index = GetIndexFromS(start_s);
    int   end_interpolation_index =
                std::fmin(num_segments_, GetIndexFromS(end_s) + 1);
    int min_index = start_interpolation_index;

    for(int i = start_interpolation_index; i < end_interpolation_index; ++i) {
      const double distance = segments_[i].DistanceSquareTo(point);
      if (distance < min_distance) {
        min_index = i;
        min_distance = distance;
      }
    }

    min_distance = std::sqrt(min_distance);
    const auto& nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);
    if (min_index == 0) {
      s = std::min(proj, nearest_seg.length());
      if (proj < 0) {
        l = prod;
      } else {
        l = (prod > 0.0 ? 1 : -1) * min_distance;
      }
    } else if (min_index == num_segments_ - 1) {
      s = accumulated_s_[min_index] + std::max(0.0, proj);
      if (proj > 0) {
        l = prod;
      } else {
        l = (prod > 0.0 ? 1 : -1) * min_distance;
      }
    } else {
      s = accumulated_s_[min_index] +
                      std::max(0.0, std::min(proj, nearest_seg.length()));
      l = (prod > 0.0 ? 1 : -1) * min_distance;
    }
    return true;

}

int RouteSegment::GetIndexFromS(double s) const{

  if(s < accumulated_s_.front()){
      return 0;
    }else if(s > accumulated_s_.back()) {
      return accumulated_s_.back();
    }

  for(int i = 1 ; i < accumulated_s_.size(); i ++ ){

      if( accumulated_s_.at(i) > s){
          return i-1;
        }

    }
}

bool RouteSegment::GetRoadWidth(const double &s,
                                      double &road_left_width,
                                      double &road_right_width) const{

    return false;
}

//const std::vector<PathOverlap>& signal_overlaps() const
const hdmap::TrafficLight & RouteSegment::SignalLightOverlaps() const{

   return route_.traffic_light;
 }

const int32_t &RouteSegment::Id() const { return id_; }

bool RouteSegment::IsOnSegment() const { return is_on_segment_; }

//bool RouteSegment::StopForDestination() const { return stop_for_destination_; }

} //namespace hdmap
