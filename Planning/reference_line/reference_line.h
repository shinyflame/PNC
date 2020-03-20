

#ifndef REFERENCE_LINE_H
#define REFERENCE_LINE_H

#include <string>
#include <utility>
#include <vector>
#include "../common/struct.h"
#include "../math/vec2d.h"
#include "../map/route_segment.h"
#include "../map/map_struct.h"

namespace planning {


class ReferenceLine
{
public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;

 // template <typename Iterator>
//  explicit ReferenceLine(const Iterator begin, const Iterator end)
//      : reference_points_(begin, end),
//        map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}

  explicit ReferenceLine(const std::vector<PathPoint>& reference_points,
                         hdmap::RouteSegment route,
                         hdmap::ReferenLineWithParam ref_line_with_param);
  //explicit ReferenceLine(const hdmap::Path& hdmap_path);


  bool GetLaneWidth(const double s, double &lane_left_width, double &lane_right_width)const ;

  void SetPriority(uint32_t priority) { priority_ = priority; }

  bool Shrink(const Vec2d& point,double look_backward, double look_forward) ;

  bool XYToSL(const Vec2d& xy_point,SLPoint* const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, SLPoint* const sl_point) const {
     return XYToSL(Vec2d(xy.x, xy.y), sl_point);
   }
  bool SLToXY(const SLPoint& sl_point, Vec2d* const xy_point) const ;

  PathPoint GetReferencePoint(const double s) const;
  int GetReferenceMatchPointIndex(const double s) const;
  const std::vector<PathPoint>& reference_points() const;

  double GetSpeedLimitFromS(const double s) const;
  double Length() const { return reference_points_.back().s; }

  bool GetApproximateSLBoundary(const Box2d& box,
                                 const double start_s, const double end_s,
                                 SLBoundary* const sl_boundary) const;
  bool IsBlockRoad(const Box2d& box2d, double gap) const;
  bool IsOnLane(const SLBoundary& sl_boundary) const ;
  bool IsOnLane(const SLPoint& sl_point) const;
  bool IsOnLane(const Vec2d& vec2d_point) const;
  template <class XYPoint>
    bool IsOnLane(const XYPoint& xy) const {
      return IsOnLane(Vec2d(xy.x, xy.y));
    }

  /**
    * @brief: check if a box/point is on road
    *         (not on sideways/medians) along reference line
    */

  bool GetRoadWidth(const double s, double &road_left_width,double &road_right_width) const;
  bool IsOnRoad(const SLPoint& sl_point) const;
  bool IsOnRoad(const Vec2d& vec2d_point) const;
  bool IsOnRoad(const SLBoundary& sl_boundary) const;

  const hdmap::RouteSegment& MapRoute() const;//using in signal light
  //PathPoint GetNearestReferencePoint(const double s) const;
  bool GetSLBoundary(const Box2d& box,SLBoundary* const sl_boundary) const;
//  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
//                                                  const double end_s) const;
  PathPoint InterpolateWithMatchedIndex(
      const PathPoint& p0, const double s0, const PathPoint& p1,
      const double s1, const double& s) const;
  hdmap::MapPoint GetMatchPoint(double s) const{ return route_.GetMatchPoint(s); }
  int GetReferenceLineId() const {return reference_line_id_ ;}

  hdmap::ReferenLineWithParam GetReferenceLineWithParam() const
  { return ref_line_with_param_;}

private:

   uint32_t priority_ = 0;
   std::vector<PathPoint> reference_points_;
   hdmap::RouteSegment route_;
   int reference_line_id_ = 0 ;
   hdmap::ReferenLineWithParam ref_line_with_param_;
  // hdmap::Path map_path_;

 };


} //end namespace planing


#endif // REFERENCE_LINE_H
