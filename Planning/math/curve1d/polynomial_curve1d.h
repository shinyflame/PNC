
/**
 * @file polynomial_curve1d.h
 **/

#ifndef MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
#define MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_

#include "../../math/curve1d/curve1d.h"

namespace planning {

class PolynomialCurve1d : public Curve1d {
 public:
  PolynomialCurve1d() = default;
  virtual ~PolynomialCurve1d() = default;

 protected:
  double param_ = 0.0;
};

}  // namespace planning


#endif  // MODULES_PLANNING_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
