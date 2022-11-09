#ifndef AVYANA_UTILS__MATH__NORMALIZATION_HPP_
#define AVYANA_UTILS__MATH__NORMALIZATION_HPP_

#include "lanelet2_extension/math/constants.hpp"
#include <cmath>

namespace avyana_utils
{
inline double normalize_degree(const double deg, const double min_deg = -180)
{
  const auto max_deg = min_deg + 360.0;

  const auto value = std::fmod(deg, 360.0);
  if (min_deg <= value && value < max_deg) {
    return value;
  }

  return value - std::copysign(360.0, value);
}

inline double normalize_radian(const double rad, const double min_rad = -pi)
{
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}

}  // namespace avyana_utils

#endif  // AVYANA_UTILS__MATH__NORMALIZATION_HPP_
