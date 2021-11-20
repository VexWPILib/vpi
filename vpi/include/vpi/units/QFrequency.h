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
#include "vpi/units/RQuantity.h"

#pragma once

namespace vpi {
QUANTITY_TYPE(0, 0, -1, 0, 0, QFrequency)

constexpr QFrequency Hz(1.0);

inline namespace literals {
constexpr QFrequency operator"" _Hz(long double x) {
  return QFrequency(x);
}
constexpr QFrequency operator"" _Hz(unsigned long long int x) {
  return QFrequency(static_cast<long double>(x));
}
} // namespace literals
} // namespace vpi
