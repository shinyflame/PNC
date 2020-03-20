
#include "../math/util.h"
#include <cmath>
#include <vector>

namespace math {

namespace util {

SLPoint MakeSLPoint(const double s, const double l) {
  SLPoint sl;
  sl.s=s;
  sl.l=l;
  return sl;
}

PointENU MakePointENU(const double x, const double y, const double z) {
  PointENU point_enu;
  point_enu.x=x;
  point_enu.y=y;
  point_enu.z=z;
  return point_enu;
}

PointENU operator+(const PointENU enu, const Vec2d& xy) {
  PointENU point;
  point.x=enu.x + xy.x();
  point.y=enu.y + xy.y();
  point.z=enu.z;
  return point;
}

PointENU MakePointENU(const Vec2d& xy) {
  PointENU point_enu;
  point_enu.x=xy.x();
  point_enu.y=xy.y();
  point_enu.z=0.0;
  return point_enu;
}

perception::Point MakePerceptionPoint(const double x, const double y,
                                              const double z) {
  perception::Point point3d;
  point3d.x=x;
  point3d.y=y;
  point3d.z=z;
  return point3d;
}

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da) {
  SpeedPoint speed_point;
  speed_point.s=(s);
  speed_point.t=(t);
  speed_point.v=(v);
  speed_point.a=(a);
  speed_point.da=(da);
  return speed_point;
}

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa) {
  PathPoint path_point;
  path_point.x=(x);
  path_point.y=(y);
  path_point.z=(z);
  path_point.theta=(theta);
  path_point.kappa=(kappa);
  path_point.dkappa=(dkappa);
  path_point.ddkappa=(ddkappa);
  return path_point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.x=(p1.x * w1 + p2.x * w2);
  p.y=(p1.y * w1 + p2.y * w2);
  p.z=(p1.z * w1 + p2.z * w2);
  p.theta=(p1.theta * w1 + p2.theta * w2);
  p.kappa=(p1.kappa * w1 + p2.kappa * w2);
  p.dkappa=(p1.dkappa * w1 + p2.dkappa * w2);
  p.ddkappa=(p1.ddkappa * w1 + p2.ddkappa * w2);
  p.s=(p1.s * w1 + p2.s * w2);
  return p;
}

}  // namespace util
} //math
