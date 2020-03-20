
/**
 * @file speed_data.h
 **/
#ifndef PLANNING_COMMON_SPEED_SPEED_DATA_H_
#define PLANNING_COMMON_SPEED_SPEED_DATA_H_

#include <string>
#include <vector>
#include "../../../Planning/common/struct.h"

namespace planning {

class SpeedData {
 public:
  SpeedData() = default;

  explicit SpeedData(std::vector<SpeedPoint> speed_points);

  virtual ~SpeedData() = default;

  const std::vector<SpeedPoint>& speed_vector() const;

  void set_speed_vector(std::vector<SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      SpeedPoint* const speed_point) const;

  double TotalTime() const;

  bool Empty() const { return speed_vector_.empty(); }

  void Clear();

  //virtual std::string DebugString() const;

 private:
  std::vector<SpeedPoint> speed_vector_;
};

}  // namespace planning


#endif  // PLANNING_COMMON_SPEED_SPEED_DATA_H_
