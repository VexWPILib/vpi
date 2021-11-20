// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

/*
 * This code is a modified version of Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "vpi/units/RQuantity.h"
#include <cmath>

namespace vpi {
QUANTITY_TYPE(0, 0, 0, 1, 0, QAngle)

constexpr QAngle radian(1.0);
constexpr QAngle degree = static_cast<double>(1_pi / 180.0) * radian;

inline namespace literals {
constexpr QAngle operator"" _rad(long double x) {
  return QAngle(x);
}
constexpr QAngle operator"" _rad(unsigned long long int x) {
  return QAngle(static_cast<double>(x));
}
constexpr QAngle operator"" _deg(long double x) {
  return static_cast<double>(x) * degree;
}
constexpr QAngle operator"" _deg(unsigned long long int x) {
  return static_cast<double>(x) * degree;
}
} // namespace literals

} // namespace vpi
