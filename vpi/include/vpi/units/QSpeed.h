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

#include "vpi/units/QLength.h"
#include "vpi/units/QTime.h"
#include "vpi/units/RQuantity.h"

namespace vpi {
QUANTITY_TYPE(0, 1, -1, 0, 0, QSpeed)

constexpr QSpeed mps = meter / second;
constexpr QSpeed miph = mile / hour;
constexpr QSpeed kmph = kilometer / hour;
constexpr QSpeed ftps = foot / second;

inline namespace literals {
constexpr QSpeed operator"" _mps(long double x) {
  return static_cast<double>(x) * mps;
}
constexpr QSpeed operator"" _ftps(long double x) {
  return static_cast<double>(x) * foot / second;
}
constexpr QSpeed operator"" _inps(long double x) {
  return static_cast<double>(x) * inch / second;
}
constexpr QSpeed operator"" _miph(long double x) {
  return static_cast<double>(x) * mile / hour;
}
constexpr QSpeed operator"" _kmph(long double x) {
  return static_cast<double>(x) * kilometer / hour;
}
constexpr QSpeed operator"" _mps(unsigned long long int x) {
  return static_cast<double>(x) * mps;
}
constexpr QSpeed operator"" _ftps(unsigned long long int x) {
  return static_cast<double>(x) * foot / second;
}
constexpr QSpeed operator"" _miph(unsigned long long int x) {
  return static_cast<double>(x) * mile / hour;
}
constexpr QSpeed operator"" _kmph(unsigned long long int x) {
  return static_cast<double>(x) * kilometer / hour;
}
} // namespace literals
} // namespace vpi
