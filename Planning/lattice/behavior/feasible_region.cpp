

#include "../../lattice/behavior/feasible_region.h"
#include <iostream>
#include <cmath>

using namespace std;
extern planning::ConfigParam g_config_param;
namespace planning {




FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s) {
  init_s_ = init_s;

  double v = init_s[1];
  //CHECK_GE(v, 0.0);

  const double max_deceleration = -g_config_param.longitudinal_acceleration_lower_bound;
  if(max_deceleration == 0)
    cout<<"ERROR max_deceleration = "<< max_deceleration<<endl;
  t_at_zero_speed_ = v / max_deceleration;
  s_at_zero_speed_ = init_s[0] + v * v / (2.0 * max_deceleration);
}

double FeasibleRegion::SUpper(const double t) const {
 // CHECK(t >= 0.0);
  return init_s_[0] + init_s_[1] * t +
      0.5*g_config_param.longitudinal_acceleration_upper_bound*t*t;

}

double FeasibleRegion::SLower(const double t) const {
  if (t < t_at_zero_speed_) {
    return init_s_[0] + init_s_[1] * t +
           0.5 * g_config_param.longitudinal_acceleration_lower_bound * t * t;

  }
  //cout<<"s_at_zero_length_ = "<<s_at_zero_speed_<<endl;
  return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
  return init_s_[1] + g_config_param.longitudinal_acceleration_upper_bound * t;
}

double FeasibleRegion::VLower(const double t) const {
  return t < t_at_zero_speed_
             ? init_s_[1] + g_config_param.longitudinal_acceleration_lower_bound * t
             : 0.0;
}

double FeasibleRegion::TLower(const double s) const {
  //CHECK(s >= init_s_[0]);

  double delta_s = s - init_s_[0];
  double v = init_s_[1];
  double a = g_config_param.longitudinal_acceleration_upper_bound;
  double t = (std::sqrt(v * v + 2.0 * a * delta_s) - v) / a;
  return t;
}


void FeasibleRegion::debug()
{

  cout<<"//*************FeasibleRegion Test ********************//"<<endl;
  cout<<"FeasibleRegion::SUpper(const double t = 8) = "<<SUpper(8)<<endl;
  cout<<"FeasibleRegion::SLower(const double t = 1) = "<<SLower(5)<<endl;
  cout<<"FeasibleRegion::VUpper(const double t = 8) = "<<VUpper(8)<<endl;
  cout<<"FeasibleRegion::VLower(const double t = 8) = "<<VLower(8)<<endl;
  cout<<"FeasibleRegion::TLower(const double s =50) = "<<TLower(50)<<endl;


}


} //end namespace planning
