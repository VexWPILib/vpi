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

// This unit is needed by the SimpleMotorFeedForward:
//   https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/native/include/frc/controller/SimpleMotorFeedforward.h

// https://en.wikipedia.org/wiki/Volt - 1 Volt = 1 Joule / Coulomb
// kg * m^2 * s^-3 * A^-1
// So a Volt / (meter-per-second):
// kg * m * s^-2 * A^-1
namespace vpi {
QUANTITY_TYPE(1, 1, -2, 0, -1, QElectricPotentialPerSpeed)

constexpr QElectricPotentialPerSpeed voltpermeterpersecond(1.0); // SI base units

inline namespace literals {
constexpr QElectricPotentialPerSpeed operator"" _voltpmps(long double x) {
  return static_cast<double>(x) * voltpermeterpersecond;
}
constexpr QElectricPotentialPerSpeed operator"" _voltpmps(unsigned long long int x) {
  return static_cast<double>(x) * voltpermeterpersecond;
}
} // namespace literals
} // namespace vpi
