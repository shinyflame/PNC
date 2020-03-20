
/**
 * @file
 **/

#include <memory>
#include <unordered_map>
#include <vector>
#include "../common/g_test.h"
#include "../math/util.h"
#include "../common/frame.h"


namespace planning {

using perception::PerceptionObstacle;

class FrameTest  {
 public:
  void SetUp() {
    ExpectTrue(ReadObstacles(prediction_obstacles_));
    prediction_obstacles_.start_timestamp = 1487029724.22 * 1000;
  }
  void FrameTest_AlignPredictionTime() {
    cout<<"FrameTest_AlignPredictionTime()"<<endl;
    int first_traj_size = prediction_obstacles_.prediction_obstacle[0]
                              .trajectory[0].trajectory_point.size();

    double origin_pred_time = prediction_obstacles_.start_timestamp /1000.0;
    Frame::AlignPredictionTime(origin_pred_time + 0.1, &prediction_obstacles_);
    ExpectEQ(first_traj_size - 1, (int)prediction_obstacles_.prediction_obstacle[0]
                                       .trajectory[0]
                                       .trajectory_point.size());

    Frame::AlignPredictionTime(origin_pred_time + 0.5, &prediction_obstacles_);
    ExpectEQ(first_traj_size - 3, (int)prediction_obstacles_.prediction_obstacle[0]
                                       .trajectory[0]
                                       .trajectory_point.size());

    Frame::AlignPredictionTime(origin_pred_time + 12.0, &prediction_obstacles_);
    ExpectEQ(0, (int)prediction_obstacles_.prediction_obstacle[0]
                     .trajectory[0]
                     .trajectory_point.size());
  }
 protected:
  prediction::PredictionObstacles prediction_obstacles_;
};

void FrameTest_AlignPredictionTime() {

  FrameTest frame_test;
  frame_test.SetUp();
  frame_test.FrameTest_AlignPredictionTime();
}

}  // namespace planning

