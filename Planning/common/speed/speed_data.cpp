
/**
 * @file speed_data.cc
 **/

#include <algorithm>
#include <utility>

#include "../../common/speed/speed_data.h"
#include "../../math/linear_interpolation.h"
#include "../../math/util.h"



namespace planning {

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : speed_vector_(std::move(speed_points)) {}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  if (!speed_vector_.empty()) {
    //CHECK(speed_vector_.back().t() < time);
  }
  speed_vector_.push_back(math::util::MakeSpeedPoint(s, time, v, a, da));
}

const std::vector<SpeedPoint>& SpeedData::speed_vector() const {
  return speed_vector_;
}

void SpeedData::set_speed_vector(std::vector<SpeedPoint> speed_points) {
  speed_vector_ = std::move(speed_points);
}

bool SpeedData::EvaluateByTime(const double t,
                               SpeedPoint* const speed_point) const {
  if (speed_vector_.size() < 2) {
    return false;
  }
  if (!(speed_vector_.front().t < t + 1.0e-6 &&
        t - 1.0e-6 < speed_vector_.back().t)) {
    return false;
  }

  auto comp = [](const SpeedPoint& sp, const double t) {
    return sp.t < t;
  };

  auto it_lower =
      std::lower_bound(speed_vector_.begin(), speed_vector_.end(), t, comp);
  if (it_lower == speed_vector_.end()) {
    *speed_point = speed_vector_.back();
  } else if (it_lower == speed_vector_.begin()) {
    *speed_point = speed_vector_.front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;

    double s = math::lerp(p0.s, t0, p1.s, t1, t);
    double v = math::lerp(p0.v, t0, p1.v, t1, t);
    double a = math::lerp(p0.a, t0, p1.a, t1, t);
    double j = math::lerp(p0.da, t0, p1.da, t1, t);

    *speed_point = math::util::MakeSpeedPoint(s, t, v, a, j);
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (speed_vector_.empty()) {
    return 0.0;
  }
  return speed_vector_.back().t - speed_vector_.front().t;
}

void SpeedData::Clear() { speed_vector_.clear(); }

//std::string SpeedData::DebugString() const {
//  const auto limit =
//      std::min(speed_vector_.size(),
//               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
//  return apollo::common::util::StrCat(
//      "[\n", apollo::common::util::PrintDebugStringIter(
//                 speed_vector_.begin(), speed_vector_.begin() + limit, ",\n"),
//      "]\n");
//}

}  // namespace planning

