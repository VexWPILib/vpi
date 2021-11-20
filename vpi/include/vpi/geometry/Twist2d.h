// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/units/QAngle.h"
#include "vpi/units/QLength.h"

namespace vpi {
/**
 * A change in distance along arc since the last pose update. We can use ideas
 * from differential calculus to create new Pose2ds from a Twist2d and vise
 * versa.
 *
 * A Twist can be used to represent a difference between two poses.
 */
struct Twist2d {
  /**
   * Linear "dx" component
   */
  QLength dx = 0_m;

  /**
   * Linear "dy" component
   */
  QLength dy = 0_m;

  /**
   * Angular "dtheta" component (radians)
   */
  QAngle dtheta = 0_rad;

  Twist2d() {
    dx = 0_m;
    dy = 0_m;
    dtheta = 0_rad;
  }

  Twist2d(QLength x, QLength y, QAngle t) {
    dx = x;
    dy = y;
    dtheta = t;
  }

  /**
   * Checks equality between this Twist2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Twist2d& other) const {
    return fabs(dx.convert(meter) - other.dx.convert(meter)) < 1E-9 &&
           fabs(dy.convert(meter) - other.dy.convert(meter)) < 1E-9 &&
           fabs(dtheta.convert(radian) - other.dtheta.convert(radian)) < 1E-9;
  }

  /**
   * Checks inequality between this Twist2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Twist2d& other) const { return !operator==(other); }
};
}  // namespace vpi
