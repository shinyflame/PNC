
/**
 * @file util.h
 **/

#ifndef PLANNING_TOOLKITS_DECIDERS_UTIL_H_
#define PLANNING_TOOLKITS_DECIDERS_UTIL_H_

#include "../../common/reference_line_info.h"



double GetADCStopDeceleration(ReferenceLineInfo* const reference_line_info,
                              const double stop_line_s,
                              const double min_pass_s_distance);



#endif  // MODULES_PLANNING_TOOLKITS_DECIDERS_UTIL_H_
