
/**
 * @file
 **/

#ifndef COMMON_MATH_PATH_MATCHER_H_
#define COMMON_MATH_PATH_MATCHER_H_

#include <utility>
#include <vector>

#include "../common/struct.h"

using namespace planning;

namespace math {

class PathMatcher {
 public:
  PathMatcher() = delete;

  static PathPoint MatchToPath( const std::vector<PathPoint>& reference_line,
                                const double x, const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<PathPoint>& reference_line, const double x,
      const double y);

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                    const double s,const int start_index,const int end_index);

 private:
  static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1,
                                       const double x, const double y);
};

}  // namespace math


#endif  // MODULES_COMMON_MATH_PATH_MATCHER_H_
