
/**
 * @file st_point.h
 **/

#ifndef PLANNING_COMMON_SPEED_ST_POINT_H_
#define PLANNING_COMMON_SPEED_ST_POINT_H_

#include <string>
#include "../../math/vec2d.h"

using namespace math;

namespace planning {

class STPoint : public Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const Vec2d& vec2d_point);

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
  std::string DebugString() const;
};

}  // namespace planning


#endif  // MODULES_PLANNING_COMMON_SPEED_ST_POINT_H_
