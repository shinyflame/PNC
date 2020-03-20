
/**
 * @file
 **/

#ifndef PLANNING_LATTICE_TRAJECTORY1D_LATTICE_TRAJECTORY1D_H_
#define PLANNING_LATTICE_TRAJECTORY1D_LATTICE_TRAJECTORY1D_H_

#include <memory>
#include <string>

#include "../../math/curve1d/curve1d.h"


namespace planning {

class LatticeTrajectory1d : public Curve1d {
 public:
  explicit LatticeTrajectory1d(std::shared_ptr<Curve1d> ptr_trajectory1d);

  virtual ~LatticeTrajectory1d() = default;

  virtual double Evaluate(const std::uint32_t order, const double param) const;

  virtual double ParamLength() const;

  //virtual std::string ToString() const;

  bool has_target_position() const;

  bool has_target_velocity() const;

  bool has_target_time() const;

  double target_position() const;

  double target_velocity() const;

  double target_time() const;

  void set_target_position(double target_position);

  void set_target_velocity(double target_velocity);

  void set_target_time(double target_time);

 private:
  std::shared_ptr<Curve1d> ptr_trajectory1d_;

  double target_position_ = 0.0;

  double target_velocity_ = 0.0;

  double target_time_ = 0.0;

  bool has_target_position_ = false;

  bool has_target_velocity_ = false;

  bool has_target_time_ = false;
};

}  // namespace planning


#endif  // MODULES_PLANNING_LATTICE_TRAJECTORY1D_LATTICE_TRAJECTORY1D_H_
