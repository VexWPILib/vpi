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

#include "vpi/units/QAngle.h"
#include "vpi/units/QFrequency.h"
#include "vpi/units/QTime.h"
#include "vpi/units/RQuantity.h"

namespace vpi {
QUANTITY_TYPE(0, 0, -1, 1, 0, QAngularSpeed)

constexpr QAngularSpeed radps = radian / second;
constexpr QAngularSpeed rpm = (360 * degree) / minute;
constexpr QAngularSpeed cps = (0.01 * degree) / second; // centidegree per second
constexpr QAngularSpeed dps = degree / second; // degree per second

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static QAngularSpeed convertHertzToRadPerSec(QFrequency in) {
  return (in.convert(Hz) / 2_pi) * radps;
}
#pragma GCC diagnostic pop

inline namespace literals {
constexpr QAngularSpeed operator"" _rpm(long double x) {
  return x * rpm;
}
constexpr QAngularSpeed operator"" _rpm(unsigned long long int x) {
  return static_cast<double>(x) * rpm;
}

constexpr QAngularSpeed operator"" _radps(long double x) {
  return x * radps;
}
constexpr QAngularSpeed operator"" _radps(unsigned long long int x) {
  return static_cast<double>(x) * radps;
}

constexpr QAngularSpeed operator"" _dps(long double x) {
  return x * dps;
}
constexpr QAngularSpeed operator"" _dps(unsigned long long int x) {
  return static_cast<double>(x) * dps;
}

} // namespace literals
} // namespace vpi