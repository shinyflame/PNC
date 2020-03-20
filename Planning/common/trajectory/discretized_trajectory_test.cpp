
/**
 * @file
 **/

#include "../../common/trajectory/discretized_trajectory.h"
#include "../../config/planning_config_param_initial.h"
#include <iostream>
#include <string>
#include "../../common/g_test.h"


#if 1
namespace planning {

void basic_test_DiscretizedTrajectory() {
  cout<<"basic_test, DiscretizedTrajectory"<<endl;
//  const std::string path_of_standard_trajectory =
//      "modules/planning/testdata/trajectory_data/standard_trajectory.pb.txt";
  std::vector<TrajectoryPoint> trajectory;
  //ConfigParamInit(g_config_param);

  //ADCTrajectory trajectory;
//  ExpectTrue(
//      common::util::GetProtoFromFile(path_of_standard_trajectory, &trajectory));
  ExpectTrue(ReadTrajectory(trajectory));
  cout<<"Trajectory Num:"<<trajectory.size()<<endl;
  DiscretizedTrajectory discretized_trajectory(trajectory);
  ExpectEQ(discretized_trajectory.GetTemporalLength(),
                   7.9999999999999885);
  ExpectEQ(discretized_trajectory.GetSpatialLength(),
                   44.752319202675167);
  auto p1 = discretized_trajectory.Evaluate(4.0);
  ExpectEQ(p1.path_point.x, 587263.01182131236);
  ExpectEQ(p1.path_point.y, 4140966.5720794979);
  ExpectEQ(p1.relative_time, 4.0);
  ExpectEQ(p1.v, 5.4412586837131443);

  int k1 = discretized_trajectory.QueryLowerBoundPoint(2.12);
  ExpectEQ(k1, 62);

  int k2 = discretized_trajectory.QueryNearestPoint({587264.0, 4140966.2});
  ExpectEQ(k2, 80);

  ExpectEQ((int)discretized_trajectory.NumOfPoints(), 121);
}

}  // namespace planning
#endif
