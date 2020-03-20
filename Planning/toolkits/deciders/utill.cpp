
/**
 * @file util.cc
 **/

#include <limits>

#include "../../toolkits/deciders/utill.h"

extern ConfigParam g_config_param;
extern VehicleParam g_vehicle_config;




double GetADCStopDeceleration(ReferenceLineInfo* const reference_line_info,
                                                 const double stop_line_s,
                                                 const double min_pass_s_distance)
{
  double adc_speed = reference_line_info->GetVehicleState().linear_velocity;

  if (adc_speed < g_config_param.max_stop_speed) {// 0.2 m/s
    return 0.0;
  }


  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s;
  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  } else {
    stop_distance = stop_line_s + min_pass_s_distance - adc_front_edge_s;//
  }

  if (stop_distance < 0.3) {
    return 0;
  }

  return (adc_speed * adc_speed) / (2 * stop_distance);
}



