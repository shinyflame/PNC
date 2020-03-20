#ifndef FEASIBLE_REGION_H
#define FEASIBLE_REGION_H

#include <algorithm>
#include <array>
#include "../../common/struct.h"

namespace  planning {



class FeasibleRegion
{
public:
  explicit FeasibleRegion(const std::array<double, 3>& init_s);

  double SUpper(const double t) const;

    double SLower(const double t) const;

    double VUpper(const double t) const;

    double VLower(const double t) const;

    double TLower(const double s) const;

    void debug();

   private:
    std::array<double, 3> init_s_;

    double t_at_zero_speed_;//velocity decrease to 0.0 m/s cost time

    double s_at_zero_speed_;
};


} //end namespace planning

#endif // FEASIBLE_REGION_H
