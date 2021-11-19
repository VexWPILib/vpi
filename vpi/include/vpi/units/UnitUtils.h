// Copyright (c) VexWPIApi contributors.
// Open VexWPIApi Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"

#include "vpi/units/RQuantity.h"
#include "vpi/units/QAngle.h"
#include "vpi/units/QAngularAcceleration.h"
#include "vpi/units/QAngularSpeed.h"
#include "vpi/units/QLength.h"
#include "vpi/units/QSpeed.h"

namespace vpi {
  class UnitUtils {
    public:
      static QAngularSpeed convertLinearSpeedToRotationalSpeed(QSpeed linear, QLength diameter, double gearRatio = 1.0) {
        return (linear * (360_deg / (diameter * 1_pi))) / gearRatio;
      }

      static QAngle convertLinearToRotational(QLength linear, QLength diameter, double gearRatio = 1.0) {
        return (linear * (360_deg / (diameter * 1_pi))) / gearRatio;
      }

      static QSpeed convertRotationalSpeedToLinearSpeed(QAngularSpeed a, QLength diameter, double gearRatio = 1.0) {
        return (a.convert(radps) * diameter.convert(meter) / 2.0) * gearRatio * mps;
      }

      static QLength convertRotationToDistance(QAngle a, QLength diameter, double gearRatio = 1.0) {
        return (a.convert(degree) / 360.0 * 1_pi * diameter.convert(meter)) * gearRatio * meter;
      }

      static QAngle convertDistanceToRotation(QLength d, QLength diameter, double gearRatio = 1.0) {
        return ((d.convert(meter) / diameter.convert(meter)) / 1_pi) / gearRatio * 360_deg;
      }

      static QAngle constrainTo360(QAngle theta) {
        return theta - 360_deg * std::floor(theta.convert(degree) / 360.0);
      }

      static QAngle constrainTo180(QAngle theta) {
        return theta - 360_deg * std::floor((theta.convert(degree) + 180.0) / 360.0);
      }
    
      static QTime now() {
        return Brain.timer(vex::msec) * millisecond;
      }
  };
} // end vpi