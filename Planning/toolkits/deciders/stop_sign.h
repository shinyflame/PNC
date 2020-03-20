
/**
 * @file
 **/

#ifndef PLANNING_TOOLKITS_DECIDERS_STOP_SIGN_H_
#define PLANNING_TOOLKITS_DECIDERS_STOP_SIGN_H_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../../toolkits/deciders/traffic_rule.h"
#include "../../map/map_struct.h"

namespace planning {

class StopSign : public TrafficRule {


 public:
  explicit StopSign(const TrafficRuleConfig& config);
  virtual ~StopSign() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

  bool FindBusStation(ReferenceLineInfo* const reference_line_info);
  bool BuildStopDecision( Frame* const frame,
                         ReferenceLineInfo* const reference_line_info,
                         hdmap::Station*const station );
  double GetBusHaveStopTime();
  bool CompleteStop();
  int GetStationId() {return station_id;}
 private:
  static constexpr const int32_t STOP_SIGN = -1;
  static double stop_begin_time_  ;
  static bool is_stop_ ;
  static int update_station_id_;
  int station_id;
  bool adc_within_station_ = false;
  hdmap::Station station_;
  VehicleState vehicle_state_;

};

}  // namespace planning

#endif  // PLANNING_TOOLKITS_DECIDERS_STOP_SIGN_H_
