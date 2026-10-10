#pragma once
#include <algorithm>
#include <cmath>

namespace ieir_controllers {
// Continuous at zero even for asymmetric Coulomb coefficients. No static
// breakaway pulse: the position servo supplies the remaining starting torque.
inline double smooth_friction(double v, double transition, double positive,
  double negative, double viscous, double gain, double limit)
{
  if (!std::isfinite(v)) {return 0.0;}
  const double sign = std::tanh(v / transition);
  const double magnitude = v >= 0.0 ? positive : negative;
  return std::clamp(gain * (sign * magnitude + viscous * v), -limit, limit);
}
inline double friction_slew(double current, double target, double rate, double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0) {return current;}
  // A late control cycle must not produce a large feedforward jump.
  const double step = rate * std::min(dt, 0.01);
  return current + std::clamp(target - current, -step, step);
}
}
