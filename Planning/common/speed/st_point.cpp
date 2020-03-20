
/**
 * @file st_point.cpp
 **/

#include "../../common/speed/st_point.h"
#include <iomanip>


namespace planning {



STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const Vec2d& vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { return set_y(s); }

void STPoint::set_t(const double t) { return set_x(t); }

std::string STPoint::DebugString() const {
 //std::string do_fraction(long double value, int decplaces = 3)
  std::string s = "s: " + std::to_string(STPoint::s());
  std::string t = "t: " + std::to_string(STPoint::t());
 return  s+t;
}

}  // namespace planning

