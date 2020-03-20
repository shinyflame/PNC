#ifndef FRAME_H
#define FRAME_H

#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "../common/obstacle.h"
#include "../common/reference_line_info.h"
//#include "../common/trajectory/publishable_trajectory.h"
#include "../reference_line/reference_line_provider.h"
#include "../common/get_data.h"
#include "../common/lag_prediction.h"
#include "../common/struct.h"

namespace planning {
class Frame
{

public:
  explicit Frame( uint32_t sequence_num,
                  const TrajectoryPoint &planning_start_point,
                  const double start_time,
                  const VehicleState &vehicle_state,
                  ReferenceLineProvider *reference_line_provider,
                  const vector<Garbage> &Garbages);

    const TrajectoryPoint &PlanningStartPoint() const;

    const VehicleState &vehicle_state() const;
    common::Status Init();

    uint32_t SequenceNum() const;

    //std::string DebugString() const;

    //const PublishableTrajectory &ComputedTrajectory() const;

    //void RecordInputDebug(planning_internal::Debug *debug);

    std::list<ReferenceLineInfo> &reference_line_info();

    Obstacle *Find(const int32_t &id);

    const ReferenceLineInfo *FindDriveReferenceLineInfo();

    const ReferenceLineInfo *DriveReferenceLineInfo() const;

    const std::vector<const Obstacle *> obstacles() const;

    const Obstacle *CreateStopObstacle(ReferenceLineInfo *const reference_line_info,
                                       const int32_t &obstacle_id,
                                       const double obstacle_s);

    const Obstacle *CreateStopObstacle(const int32_t &obstacle_id,
                                       const int32_t &lane_id,
                                       const double lane_s  );

    const Obstacle *CreateStaticObstacle( ReferenceLineInfo *const reference_line_info,
                                          const int32_t &obstacle_id,
                                          const double obstacle_start_s,
                                          const double obstacle_end_s);
    bool Rerouting();
    static void AlignPredictionTime(const double planning_start_time,
                              prediction::PredictionObstacles *prediction_obstacles);
    const bool is_near_destination() const { return is_near_destination_; }

        /**
         * @brief Adjust reference line priority according to actual road conditions
         * @id_to_priority lane id and reference line priority mapping relationship
         */
    void UpdateReferenceLinePriority(const std::map<int32_t, uint32_t> &id_to_priority);
    void SetCommand(const SoftCommand &command){command_ = command;}
    const SoftCommand GetCommand() {return command_;}
    ReferenceLineProvider &reference_line_provider(){
        return *reference_line_provider_;
    }
    RouteType GetRouteType() {   return route_type_;}
    bool NoUsingRightBackUltra() { return no_using_right_back_ultra_;}
private:
    bool CreateReferenceLineInfo();

    const Obstacle *CreateStaticVirtualObstacle(const int32_t &id, const Box2d &box);

    const Obstacle *FindCollisionObstacle() const;

    void AddObstacle(const Obstacle &obstacle);



private:
    uint32_t sequence_num_ = 0;
    const hdmap::HDMap *hdmap_ = nullptr;
    TrajectoryPoint planning_start_point_;
    const double start_time_ = 0.0;
    VehicleState vehicle_state_;
    std::list<ReferenceLineInfo> reference_line_info_;
    bool is_near_destination_ = false;
    ThreadSafeIndexedObstacles obstacles_;
    //ChangeLaneDecider change_lane_decider_;
    /**
     * the reference line info that the vehicle finally choose to drive on
    **/
    const ReferenceLineInfo *drive_reference_line_info_ = nullptr;
    prediction::PredictionObstacles prediction_;
    std::unique_ptr<LagPrediction> lag_predictor_;
    ReferenceLineProvider *reference_line_provider_ = nullptr;
    vector<Garbage> garbages_;
    SoftCommand command_;
    RouteType route_type_;
    bool no_using_right_back_ultra_;
};

void FrameTest_AlignPredictionTime();

} //namespace planning
#endif // FRAME_H
