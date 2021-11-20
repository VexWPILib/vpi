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
QUANTITY_TYPE(0, 1, -3, 0, 0, QJerk)

constexpr QJerk mps3 = meter / (second * second * second);
constexpr QJerk ftps3 = foot / (second * second * second);

inline namespace literals {
constexpr QJerk operator"" _mps3(long double x) {
  return QJerk(x);
}
constexpr QJerk operator"" _mps3(unsigned long long int x) {
  return QJerk(static_cast<double>(x));
}
} // namespace literals

}
