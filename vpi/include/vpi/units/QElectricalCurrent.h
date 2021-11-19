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

namespace vpi {
QUANTITY_TYPE(0, 0, -1, 0, 1, QElectricalCurrent)

// https://en.wikipedia.org/wiki/Ampere - 1 Amp = 1 Coulomb / second
constexpr QElectricalCurrent ampere(1.0); // SI base unit

inline namespace literals {
constexpr QElectricalCurrent operator"" _amp(long double x) {
  return static_cast<double>(x) * ampere;
}
constexpr QElectricalCurrent operator"" _amp(unsigned long long int x) {
  return static_cast<double>(x) * ampere;
}
} // namespace literals
} // namespace vpi
