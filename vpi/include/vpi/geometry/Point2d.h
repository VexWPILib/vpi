// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <cmath>

#include "vpi/units/QAngle.h"
#include "vpi/units/QLength.h"

namespace vpi {
class Point2d {
  QLength x{0_m};
  QLength y{0_m};

private:
  Point2d computeDiffs(const Point2d &ipoint) {
    Point2d retval((x - ipoint.x),(y - ipoint.y));
    return retval;
  }

  QLength computeDistance(QLength xDiff, QLength yDiff) {
    return ((xDiff.convert(meter) * xDiff.convert(meter)) + (yDiff.convert(meter) * yDiff.convert(meter))) * meter;
  }

public:
  Point2d(QLength myx, QLength myy) {
    x = myx;
    y = myy;
  }

  QLength computeDistanceToPoint(const Point2d &ipoint) {
    Point2d d = computeDiffs(ipoint);
    return computeDistance(d.x, d.y);
  }

  QAngle computeAngleToPoint(const Point2d &ipoint) {
    Point2d d = computeDiffs(ipoint);
    return std::atan2(d.y.convert(meter), d.y.convert(meter)) * radian;
  }
};
} // namespace vpi
