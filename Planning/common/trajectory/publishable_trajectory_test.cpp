
/**
 * @file
 **/

#include "../../common/trajectory/publishable_trajectory.h"
#include <string>
#include "../../common/g_test.h"


namespace planning {

void basic_test_PublishableTrajectory() {
//  const std::string path_of_standard_trajectory =
//      "modules/planning/testdata/trajectory_data/standard_trajectory.pb.txt";
  std::vector<TrajectoryPoint> trajectory;
//  trajectory.gear = GEAR_DRIVE;
//  trajectory.is_replan = true;
//  trajectory.total_path_length = 44.752319202675167;
//  trajectory.total_path_time = 7.9999999999999885;
//  trajectory.right_of_way_status = UNPROTECTED;
//  trajectory.engage_advice.advice = KEEP_ENGAGED;

  ExpectTrue(ReadTrajectory(trajectory));

  DiscretizedTrajectory discretized_trajectory(trajectory);

  PublishableTrajectory publishable_trajectory(12349834.26,
                                               discretized_trajectory);
  ExpectEQ(publishable_trajectory.header_time(), 12349834.26);

  ADCTrajectory output_trajectory;
  publishable_trajectory.PopulateTrajectoryProtobuf(&output_trajectory);

  //google::protobuf::util::MessageDifferencer differencer;
  for (int i = 0; i < output_trajectory.trajectory_point.size(); ++i) {
    if(output_trajectory.trajectory_point.at(i).relative_time !=
                               trajectory.at(i).relative_time  )
      {cout<<"Error"<<" i = "<<i<<" "<<endl;}
  }
}

}  // namespace planning

