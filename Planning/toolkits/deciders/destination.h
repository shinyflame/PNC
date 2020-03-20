#ifndef DESTINATION_H
#define DESTINATION_H

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "../../toolkits/deciders/traffic_rule.h"
#include "../../map/map_struct.h"

namespace planning {

class Destination : public TrafficRule {


 public:
  explicit Destination(const TrafficRuleConfig& config);
  virtual ~Destination() = default;

  common::Status ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);

  bool FindDestination(ReferenceLineInfo* const reference_line_info);
  bool BuildStopDecision( Frame* const frame,
                         ReferenceLineInfo* const reference_line_info,
                         hdmap::Terminal*const destination );



 private:
  static constexpr const int32_t STOP_SIGN = -1;

  hdmap::Terminal destination_;
  VehicleState vehicle_state_;

};

}  // namespace planning


#endif // DESTINATION_H
