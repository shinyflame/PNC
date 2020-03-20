
#include "reference_line.h"

#include <iostream>
#include <algorithm>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "../math/angle.h"
#include "../math/linear_interpolation.h"
#include "../math/vec2d.h"
#include "../math/util.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;

using namespace std;

namespace planning {



ReferenceLine::
ReferenceLine(const std::vector<PathPoint>& reference_points,
              hdmap::RouteSegment route,
              hdmap::ReferenLineWithParam ref_line_with_param)
             : reference_points_(reference_points),route_(route),
               reference_line_id_(route.GetRouteId()),
               ref_line_with_param_(ref_line_with_param){ }


bool ReferenceLine::GetLaneWidth(const double s, double &lane_left_width,
                                 double &lane_right_width) const {
    route_.GetLaneWidth(s,lane_left_width,lane_right_width);
   //lane_left_width = 3;lane_right_width = 3;
    return true;

}

bool ReferenceLine::XYToSL(const Vec2d& xy_point, SLPoint* const sl_point) const {

  //DCHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;
  //if (!map_path_.GetProjection(xy_point, &s, &l)) {
  if (!route_.GetProjection(xy_point, s, l)) {
    cout << "Can't get nearest point from path."<<endl;
    return false;
  }
  sl_point->s = s;
  sl_point->l = l;
  return true;

}

bool ReferenceLine::SLToXY(const SLPoint& sl_point, Vec2d* const xy_point) const {

  //CHECK_NOTNULL(xy_point);
   if (reference_points_.size() < 2) {
      cout << "Error The reference line has too few points."<<endl;
      return false;
    }

    const auto matched_point = GetReferencePoint(sl_point.s);
    const auto angle = matched_point.theta + M_PI_2;
    xy_point->set_x(matched_point.x - sin(angle) * sl_point.l);
    xy_point->set_y(matched_point.y + cos(angle) * sl_point.l);

    return true;
}

PathPoint ReferenceLine::GetReferencePoint(const double s) const {

    //const auto& accumulated_s = route_.accumulated_s(s);
    if (s < reference_points_.front().s - 1e-2) {
      cout << "AWARN The requested s " << s << " < 0"<<endl;
      return reference_points_.front();
    }
    if (s >reference_points_.back().s + 1e-2) {
      cout << "AWARN The requested s " << s << " > reference line length "
           << reference_points_.back().s <<endl;
      return reference_points_.back();
    }

    uint32_t index = 0 ;
    uint32_t pre_index = 0;
    for(int i = 0; i < reference_points_.size(); i++ )
      {
        if(reference_points_.at(i).s > s){
            index = i;
            pre_index = i - 1;
            break;
          }
      }

    if( pre_index < 0 ) {
      pre_index = 0;
    }

    const auto& p0 = reference_points_[pre_index];
    const auto& p1 = reference_points_[index];

    const double s0 = p0.s;
    const double s1 = p1.s - p0.s;
    double s2 = s - s0 ;

    return InterpolateWithMatchedIndex(p0, 0.0, p1, s1, s2 );

}

int ReferenceLine::GetReferenceMatchPointIndex(const double s) const{

  if (s < reference_points_.front().s - 1e-2) {
    cout << "AWARN The requested s " << s << " < 0"<<endl;
    return 0;
  }
  if (s >reference_points_.back().s + 1e-2) {
    cout << "AWARN The requested s " << s << " > reference line length "
         << reference_points_.back().s <<endl;
    return reference_points_.size() - 1;
  }

  uint32_t index = 0 ;
  uint32_t pre_index = 0;
  for(int i = 0; i < reference_points_.size(); i++ )
    {
      if(reference_points_.at(i).s > s){
          index = i;
          pre_index = i - 1;
          break;
        }
    }

  if( pre_index < 0 ) {
    pre_index = 0;
  }

  return pre_index;

}

PathPoint ReferenceLine::InterpolateWithMatchedIndex(
    const PathPoint& p0, const double s0, const PathPoint& p1,
    const double s1, const double& s) const {
  if (std::fabs(s0 - s1) < kMathEpsilon) {
    return p0;
  }
  PathPoint out_point;
  out_point.x = math::lerp(p0.x, s0, p1.x, s1, s);
  out_point.y = math::lerp(p0.y, s0, p1.y, s1, s);
  out_point.z = math::lerp(p0.z, s0, p1.z, s1, s);
  out_point.s = math::lerp(p0.s, s0, p1.s, s1, s);
  out_point.theta = math::slerp(p0.theta, s0, p1.theta, s1, s);
  out_point.kappa = math::lerp(p0.kappa , s0, p1.kappa , s1, s);
  out_point.dkappa = math::lerp(p0.dkappa, s0, p1.dkappa, s1, s);
  out_point.ddkappa = math::lerp(p0.ddkappa, s0, p1.ddkappa, s1, s);

  return out_point;
}

const std::vector<PathPoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}


double ReferenceLine::GetSpeedLimitFromS(const double s) const {


  double speed_limit = route_.RouteSpeedLimitFromS(s);
  if( speed_limit > g_config_param.planning_upper_speed_limit)//11.1
    {
      speed_limit = g_config_param.planning_upper_speed_limit ;
    }

  return g_config_param.planning_upper_speed_limit;
}

bool ReferenceLine::GetApproximateSLBoundary(const Box2d& box,
                               const double start_s, const double end_s,
                              SLBoundary* const sl_boundary) const{

    double s = 0.0;
    double l = 0.0;
    double distance = 0.0;
    if (!route_.GetProjectionWithHeuristicParams(box.center(), start_s, end_s,
                                                    s, l, distance)) {
      cout << "Error Can't get projection point from path."<<endl;
      return false;
    }

    auto projected_point = GetReferencePoint(s);
    auto rotated_box = box;
    rotated_box.RotateFromCenter(-projected_point.theta);

    std::vector<Vec2d> corners;
    rotated_box.GetAllCorners(&corners);

    double min_s(std::numeric_limits<double>::max());
    double max_s(std::numeric_limits<double>::lowest());
    double min_l(std::numeric_limits<double>::max());
    double max_l(std::numeric_limits<double>::lowest());

    for (const auto& point : corners) {
      // x <--> s, y <--> l
      // because the box is rotated to align the reference line
      min_s = std::fmin(min_s, point.x() - rotated_box.center().x() + s);
      max_s = std::fmax(max_s, point.x() - rotated_box.center().x() + s);
      min_l = std::fmin(min_l, point.y() - rotated_box.center().y() + l);
      max_l = std::fmax(max_l, point.y() - rotated_box.center().y() + l);
    }
    sl_boundary->start_s = (min_s);
    sl_boundary->end_s   = (max_s);
    sl_boundary->start_l = (min_l);
    sl_boundary->end_l   = (max_l);

  return true;
}

bool ReferenceLine::GetSLBoundary(const Box2d& box, SLBoundary* const sl_boundary) const
{
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  std::vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
        std::cout << "failed to get projection for point: " //<< point.DebugString()
                  << " on reference line."<<std::endl;
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s);
    end_s = std::fmax(end_s, sl_point.s);
    start_l = std::fmin(start_l, sl_point.l);
    end_l = std::fmax(end_l, sl_point.l);
  }
  sl_boundary->start_s = start_s;
  sl_boundary->end_s   = end_s;
  sl_boundary->start_l = start_l;
  sl_boundary->end_l   = end_l;
  return true;
}

bool ReferenceLine::IsBlockRoad(const Box2d& box2d, double gap) const{

  //return map_path_.OverlapWith(box2d, gap);
  return route_.OverlapWith(box2d, gap);
}



bool ReferenceLine::IsOnLane(const SLBoundary& sl_boundary) const
{
  if (sl_boundary.end_s < 0 || sl_boundary.start_s > Length()) {
     return false;
   }
   double middle_s = (sl_boundary.start_s + sl_boundary.end_s) / 2.0;
   double lane_left_width = 0.0;
   double lane_right_width = 0.0;
   route_.GetLaneWidth(middle_s, lane_left_width, lane_right_width);

   return !(sl_boundary.start_l > lane_left_width ||
            sl_boundary.end_l < -lane_right_width);
}

bool ReferenceLine::IsOnLane(const Vec2d& vec2d_point) const {
  SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnLane(sl_point);
}

bool ReferenceLine::IsOnLane(const SLPoint& sl_point) const {

  if (sl_point.s <= 0 || sl_point.s > route_.Length()) {
      return false;
    }
    double left_width = 0.0;
    double right_width = 0.0;

    if (!GetLaneWidth(sl_point.s, left_width, right_width)) {
      return false;
    }

    return !(sl_point.l < -right_width || sl_point.l > left_width);
}


bool ReferenceLine::GetRoadWidth(const double s, double &road_left_width,
                                 double &road_right_width) const {
  if (route_.GetPathCenterPoints().size() == 0) {
    return false;
  }
  return route_.GetRoadWidth(s, road_left_width, road_right_width);
}

bool ReferenceLine::IsOnRoad(const SLBoundary& sl_boundary) const {

  if (sl_boundary.end_s < 0 || sl_boundary.start_s > Length()) {
      return false;
    }
    double middle_s = (sl_boundary.start_s + sl_boundary.end_s) / 2.0;
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    route_.GetRoadWidth(middle_s, road_left_width, road_right_width);
    return !(sl_boundary.start_l > road_left_width ||
             sl_boundary.end_l < -road_right_width);

}
bool ReferenceLine::IsOnRoad(const SLPoint& sl_point) const{

  if (sl_point.s <= 0 || sl_point.s > route_.Length()) {
      return false;
    }
    double road_left_width = 0.0;
    double road_right_width = 0.0;

    if (!GetRoadWidth(sl_point.s, road_left_width, road_right_width)) {
      return false;
    }

    return !(sl_point.l < -road_right_width || sl_point.l > road_left_width);
}
bool ReferenceLine::IsOnRoad(const Vec2d& vec2d_point) const{

  SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnRoad(sl_point);

}

const hdmap::RouteSegment& ReferenceLine::MapRoute() const { return route_; }


bool ReferenceLine::Shrink(const  Vec2d& point,
                           double look_backward, double look_forward) {

  return true;
}




//std::vector<hdmap::LaneSegment> ReferenceLine::GetLaneSegments(
//    const double start_s, const double end_s) const {
//  return map_path_.GetLaneSegments(start_s, end_s);
//}

} //end namespace planing
