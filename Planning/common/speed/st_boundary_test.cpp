
#include <algorithm>
#include <cmath>

#include "../../common/speed/st_boundary.h"
#include "../../common/g_test.h"



namespace planning {

//using apollo::common::math::Box2d;
//using apollo::common::math::Vec2d;
#if 1
void StBoundaryTest_basic_test(void) {
  cout<<"basic_test"<<endl;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(0.0, 0.0);
  lower_points.emplace_back(0.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  StBoundary boundary(point_pairs);

  ExpectEQ(boundary.id(), 0);
  ExpectEQ(boundary.boundary_type(), StBoundary::BoundaryType::UNKNOWN);
  ExpectEQ(0.0, boundary.min_s());
  ExpectEQ(5.0, boundary.max_s());
  ExpectEQ(0.0, boundary.min_t());
  ExpectEQ(10.0, boundary.max_t());
}

void StBoundaryTest_boundary_range(void) {
  cout<<"Test_boundary_range"<<endl;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(1.0, 0.0);
  lower_points.emplace_back(1.0, 10.0);
  upper_points.emplace_back(5.0, 0.0);
  upper_points.emplace_back(5.0, 10.0);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  StBoundary boundary(point_pairs);

  boundary.SetBoundaryType(StBoundary::BoundaryType::YIELD);
  double t = -10.0;
  const double dt = 0.01;
  while (t < 10.0) {
    double low = 0.0;
    double high = 0.0;
    if (t < 0.0) {
      ExpectTrue(boundary.GetUnblockSRange(t, &high, &low));
      ExpectEQ(low, 0.0);
      ExpectEQ(high, 200.0);
      ExpectFalse(boundary.GetBoundarySRange(t, &high, &low));
    } else {
      ExpectTrue(boundary.GetUnblockSRange(t, &high, &low));
      ExpectEQ(low, 0.0);
      ExpectEQ(high, 1.0);

      ExpectTrue(boundary.GetBoundarySRange(t, &high, &low));
      ExpectEQ(low, 1.0);
      ExpectEQ(high, 5.0);
    }
    t += dt;
  }
}

void StBoundaryTest_get_index_range() {
  cout<<"get_index_range"<<endl;
  std::vector<STPoint> upper_points;
  std::vector<STPoint> lower_points;

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  lower_points.emplace_back(43.000164837720789, -517957.08587679861);
  lower_points.emplace_back(46.100164825451913, -517955.58587660792);

  upper_points.emplace_back(52.200164801309178, -517957.08587679861);
  upper_points.emplace_back(55.6001647283625, -517955.58587660792);

  point_pairs.emplace_back(lower_points[0], upper_points[0]);
  point_pairs.emplace_back(lower_points[1], upper_points[1]);

  StBoundary boundary(point_pairs);

  size_t left = 0;
  size_t right = 0;

  ExpectTrue(
      boundary.GetIndexRange(lower_points, -517957.08587679861, &left, &right));
  ExpectEQ((int)left, 0);
  ExpectEQ((int)right, 0);

  ExpectTrue(
      boundary.GetIndexRange(lower_points, -517955.58587660792, &left, &right));
  ExpectEQ((int)left, 0);
  ExpectEQ((int)right, 1);

  ExpectTrue(
      boundary.GetIndexRange(lower_points, -517955.58587660792, &left, &right));
  ExpectEQ((int)left, 0);
  ExpectEQ((int)right, 1);

  ExpectFalse(boundary.GetIndexRange(lower_points, 0.0, &left, &right));
}

void StBoundaryTest_remove_redundant_points() {
  cout<<"remove_redundant_points"<<endl;
  std::vector<std::pair<STPoint, STPoint>> points;
  points.emplace_back(STPoint(0.0, 0.0), STPoint(1.0, 0.0));
  points.emplace_back(STPoint(0.1, 0.2), STPoint(1.1, 0.2));
  points.emplace_back(STPoint(0.2, 0.3), STPoint(1.2, 0.3));
  points.emplace_back(STPoint(0.3, 0.4), STPoint(1.3, 0.4));
  points.emplace_back(STPoint(0.4, 0.5), STPoint(1.4, 0.5));

  ExpectEQ((int)points.size(), 5);

  StBoundary st_boundary;
  st_boundary.RemoveRedundantPoints(&points);

  ExpectEQ((int)points.size(), 2);
  ExpectEQ(points[0].first.s(), 0.0);
  ExpectEQ(points[0].first.t(), 0.0);
  ExpectEQ(points[0].second.s(), 1.0);
  ExpectEQ(points[0].second.t(), 0.0);

  ExpectEQ(points[1].first.s(), 0.4);
  ExpectEQ(points[1].first.t(), 0.5);
  ExpectEQ(points[1].second.s(), 1.4);
  ExpectEQ(points[1].second.t(), 0.5);
}
#endif
}  // namespace planning

