#ifndef ROUTE_SEGMENTS_H
#define ROUTE_SEGMENTS_H

#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "../math/vec2d.h"
#include "../common/struct.h"
#include "../map/map_struct.h"
#include "../math/box2d.h"

using namespace math;

namespace hdmap {
class RouteSegment
{

public:
  RouteSegment(Route route,int route_id, bool on_route,
               bool is_update,bool is_side_slip);

  MapPoint GetMatchPoint(double s) const;
  MapPoint AccurateInterpolation(MapPoint p0,double s0,
                                 MapPoint p1,double s1,double s) const;
  double Length() const { return route_.reference_points.back().s ; }

  bool GetLaneWidth(const double &s,double &left_width,double &right_width) const;

  bool GetProjection(const Vec2d &point, double &s, double&l) const;

  double RouteSpeedLimitFromS(const double &s) const;

  bool OverlapWith(const Box2d &box2d, double gap) const;

  const std::vector<double>& accumulated_s(const double & s) const
        { return accumulated_s_; }
  bool GetProjectionWithHeuristicParams(const  Vec2d &box_center,
                                        const double &start_s,
                                        const double &end_s,
                                              double &s,
                                              double &l,
                                              double &distance) const;
  int GetIndexFromS(double start_s) const;

  bool GetRoadWidth(const double &s, double &road_left_width,
                                     double &road_right_width) const;

  const hdmap::TrafficLight & SignalLightOverlaps() const;

  std::vector<MapPoint> GetPathCenterPoints() const { return route_.reference_points;}

  MapPoint GetMapOriginalPoint() { return map_original_point_ ;}


  bool Shrink(const Vec2d &point, const double look_backward,
                                  const double look_forward  );
  const int32_t &Id() const;

  bool StopForDestination() const { return stop_for_destination_; }

  bool IsOnSegment() const;

  Route GetRoute() const {return route_;}



  Station GetBusStation() const {return route_.station; }
  Terminal GetDestination() const { return route_.terminal; }
  void SetUpdate(bool state){ update_state_ = state;}
  bool GetUpdate() { return update_state_;}
  int  GetRouteId() const { return id_;}
  bool GetSideType() const {return is_side_slip_;}

private:

  int id_;
  Route route_;
  bool  stop_for_destination_ = false;
  std::vector<double> accumulated_s_;
   /**
      * Indicates whether the vehicle is on current RouteSegment.
   **/
   bool is_on_segment_ = false;

   MapPoint map_original_point_;
   std::vector<LineSegment2d> segments_;
   bool update_state_ = false;
   bool is_side_slip_ = false;
};
} // namespace hdmap
#endif // ROUTE_SEGMENTS_H
