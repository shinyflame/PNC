
/**
 * @file
 **/
#include <memory>
#include <utility>
#include "../common/path_decision.h"
//#include "../common/util/util.h"


namespace planning {

using IndexedPathObstacles = IndexedList<int32_t, PathObstacle>;

PathObstacle *PathDecision::AddPathObstacle(const PathObstacle &path_obstacle) {
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  return path_obstacles_.Add(path_obstacle.Id(), path_obstacle);
}

const IndexedPathObstacles &PathDecision::path_obstacles() const {
  return path_obstacles_;
}

PathObstacle *PathDecision::Find(const int32_t &object_id) {
  return path_obstacles_.Find(object_id);
}

const PathObstacle *PathDecision::Find(const int32_t &object_id) const {
  return path_obstacles_.Find(object_id);
}

void PathDecision::SetStBoundary(const int32_t &id,
                                 const StBoundary &boundary) {
  auto *obstacle = path_obstacles_.Find(id);

  if (!obstacle) {
    cout << "Failed to find obstacle : " << id<<endl;
    return;
  } else {
    obstacle->SetStBoundary(boundary);
  }
}

bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const int32_t &object_id,
                                      const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);
  if (!path_obstacle) {
    cout << "failed to find obstacle"<<endl;
    return false;
  }
  path_obstacle->AddLateralDecision(tag, decision);
  return true;
}

void PathDecision::EraseStBoundaries() {
  for (const auto *path_obstacle : path_obstacles_.Items())
  {
    auto *obstacle_ptr = path_obstacles_.Find(path_obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                           const int32_t &object_id,
                                           const ObjectDecisionType &decision) {
  auto *path_obstacle = path_obstacles_.Find(object_id);
  if (!path_obstacle) {
    cout << "failed to find obstacle"<<endl;
    return false;
  }
  path_obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const int32_t &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  PointENU stop_point = obj_stop.stop_point;
  SLPoint stop_line_sl;
  reference_line.XYToSL({stop_point.x, stop_point.y}, &stop_line_sl);

  double stop_line_s = stop_line_sl.s;
  if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
    cout << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]"<<endl;
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const double kStopBuff = 1.0;
  stop_line_s = std::fmax(stop_line_s, adc_sl_boundary.end_s - kStopBuff);

  if (stop_line_s >= stop_reference_line_s_) {
    cout << "stop point is further than current main stop point."<<endl;
    return false;
  }

  //main_stop_.Clear();
  main_stop_.reason_code = obj_stop.reason_code;
  main_stop_.reason = "stop by " + std::to_string(obj_id);
  main_stop_.stop_point.x = obj_stop.stop_point.x;
  main_stop_.stop_point.y = obj_stop.stop_point.y;
  main_stop_.stop_heading = obj_stop.stop_heading;
  stop_reference_line_s_ = stop_line_s;

  cout   << " main stop obstacle id:" << obj_id
         << " stop_line_s:" << stop_line_s << " stop_point: ("
         << obj_stop.stop_point.x <<" , "<< obj_stop.stop_point.y
         << " ) stop_heading: " << obj_stop.stop_heading<<endl;
  return true;
}

}  // namespace planning

