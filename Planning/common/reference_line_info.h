#ifndef REFERENCE_LINE_INFO_H
#define REFERENCE_LINE_INFO_H

#include <algorithm>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include "../reference_line/reference_line.h"
#include "../common/obstacle.h"
#include "../map/route_segment.h"
#include "../common/trajectory/discretized_trajectory.h"
#include "../common/path_decision.h"
#include "../common/path_decision.h"
#include "../common/speed/speed_data.h"
#include "../common/trajectory/discretized_trajectory.h"



namespace planning {


class ReferenceLineInfo
{
public:
  ReferenceLineInfo() = default;
  explicit ReferenceLineInfo(  const VehicleState& vehicle_state,
                               const TrajectoryPoint& adc_planning_point,
                               const ReferenceLine& reference_line,
                               const hdmap::RouteSegment& segments );


  const ReferenceLine& reference_line() const;

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }

  const hdmap::RouteSegment& Lanes() const;

  bool Init(const std::vector<const Obstacle*>& obstacles) ;
  bool IsInited() const;

  void SetOffsetToOtherReferenceLine(const double offset) {
      offset_to_other_reference_line_ = offset;
    }

  void SetDrivable(bool drivable);

  bool IsDrivable() const;

  double Cost() const { return cost_; }
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetSlideCleanCost(double cost) { slide_clean_cost_ = cost; }
  double SlideCleanCost() const { return slide_clean_cost_ ; }


  bool IsChangeLanePath() const;

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  void SetCruiseSpeed(double speed);
  void SetStopPoint(const StopPoint& stop_point);
  const PlanningTarget& planning_target() const { return planning_target_; }

  void SetTrajectory(const DiscretizedTrajectory& trajectory);

  const LatencyStats& latency_stats() const { return latency_stats_; }

  RightOfWayProtectedStatus GetRightOfWayStatus() const;

  const std::list<common::Id> TargetLaneId() const;

  ADC::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

  void ExportDecision(DecisionResult* decision_result) const;

  const DiscretizedTrajectory& trajectory() const;
  double TrajectoryLength() const ;

  const SLBoundary& AdcSlBoundary() const;
  const SLBoundary& VehicleSlBoundary() const;

  PathDecision* path_decision();
  const PathDecision& path_decision() const;

  PathObstacle* AddObstacle(const Obstacle* obstacle);
  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  bool IsUnrelaventObstacle(PathObstacle* path_obstacle);

  bool AddObstacleHelper(const Obstacle* obstacle);

  bool IsRightTurnPath() const;

  void SetJunctionRightOfWay(double junction_s, bool is_protected);

  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  const SpeedData& speed_data() const ;
  SpeedData* mutable_speed_data();

//  bool CombinePathAndSpeedProfile(const double relative_time, const double start_s,
//                                  DiscretizedTrajectory* ptr_discretized_trajectory);
  void MakeDecision(DecisionResult* decision_result) const ;
  int  MakeMainStopDecision(DecisionResult* decision_result) const;
  void MakeEStopDecision(DecisionResult* decision_result) const ;
  void MakeMainMissionCompleteDecision(DecisionResult* decision_result) const;
  bool ReachedDestination() const;
  void SetObjectDecisions(ObjectDecisions* object_decisions) const ;
  void ExportEngageAdvice(EngageAdvice* engage_advice) const;

  const SLPoint& GetVechicleSLPoint() const { return vehicle_sl_ ; }
  void SetGarbagesSL(vector<SLPoint> garbages_sl)
                                        { garbages_sl_ = garbages_sl; }
  const vector<SLPoint> GetGarbagesSL() const { return garbages_sl_ ; }
  const VehicleState GetVehicleState() const { return vehicle_state_; }
  const int GetRouteId() const  { return route_.GetRouteId(); }
  bool IsSideSlipClean() const  { return route_.GetSideType();}
  bool GetStopDistanceWithObstacle
  (double &stop_dis,std::vector<const Obstacle *> obstacles)const ;
  SampleRecommendation SamepleDecider(std::vector<const Obstacle *> obstacles)const ;
  SampleRecommendation SamepleDeciderWithObsSL(std::vector<const Obstacle *> obstacles)const ;
  const ReferenceLine * reference_line_mutable() {
    return &reference_line_;
  }

  SLBoundary ComputeObstacleBoundary(const std::vector<Vec2d>& vertices,
                    const std::vector<PathPoint>& discretized_ref_points) const;

  SLPoint GetSLPoint(Vec2d point) const;

private:

   bool CheckChangeLane() const;

  struct {
      /**
       * @brief SL boundary of stitching point (starting point of plan trajectory)
       * relative to the reference line
       */
      SLBoundary adc_sl_boundary_;
      /**
       * @brief SL boundary of vehicle realtime state relative to the reference
       * line
       */
      SLBoundary vehicle_sl_boundary_;
    } sl_boundary_info_;

   ReferenceLine reference_line_;
   VehicleState vehicle_state_;
   SLPoint vehicle_sl_;
   TrajectoryPoint adc_planning_point_;
   hdmap::RouteSegment route_;
   double cost_ = 0.0;
   bool   is_drivable_ = true;
   double priority_cost_ = 0.0;
   double slide_clean_cost_  = 0.0;
   double offset_to_other_reference_line_ = 0.0 ;
   bool   is_on_reference_line_ = false;
   PlanningTarget        planning_target_;
   DiscretizedTrajectory discretized_trajectory_;
   LatencyStats          latency_stats_;
   ADC::TrajectoryType   trajectory_type_ = ADC::UNKNOWN ;
   PathDecision path_decision_;
   bool is_inited_ = false;
   //bool is_safe_to_change_lane_ = false;
   SpeedData speed_data_;
   vector<SLPoint> garbages_sl_;

};
}//end namespace planning
#endif // REFERENCE_LINE_INFO_H
