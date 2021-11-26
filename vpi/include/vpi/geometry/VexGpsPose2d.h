// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/geometry/Pose2d.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {

 /**
  * WPILib assumes a coordinate system where increasing X on the 
  * horizontal axis would have a heading of 0 degrees, while the
  * Vex GPS sensor reports this as a heading of 90 degrees.  Given that
  * users would want to use their familiar co-ordinates and heading,
  * we will default to VEX_GPS, and ensure that all calls inside this
  * library use VPI_NATIVE when creating Pose2d objects.
  */
class VexGpsPose2d {
 public:
  /**
   * Constructs a pose at the origin facing toward the positive X axis.
   * (Translation2d{0, 0} and Rotation{0})
   */
  constexpr VexGpsPose2d() = default;

  /**
   * Convenience constructors that takes in x and y values directly instead of
   * having to construct a Translation2d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param t The heading component of the pose.
   */
  VexGpsPose2d(QLength x, QLength y, QAngle t) : m_x(x), m_y(y), m_theta(t) {}

  // Conversions between Pose2d and VexGpsPose2d
  // -------------------------------------------
  // Constructor
  VexGpsPose2d(const Pose2d& p) : m_x(p.X()), m_y(p.Y()),
      m_theta(UnitUtils::constrainTo180(90_deg - p.Rotation().ToAngle())) {}
  // Assignment
  VexGpsPose2d& operator=(const Pose2d& p) {
    this->m_x = p.X();
    this->m_y = p.Y();
    this->m_theta = UnitUtils::constrainTo180(90_deg - p.Rotation().ToAngle());
    return *this;
  }
   // Type-cast
  operator Pose2d() {return Pose2d(m_x, m_y, UnitUtils::constrainTo180(90_deg - m_theta));}

  const QLength X() const {return m_x;}
  const QLength Y() const {return m_y;}
  const QAngle Theta() const {return m_theta;}

 private:
  QLength m_x = 0_m;
  QLength m_y = 0_m;
  QAngle m_theta = 0_rad;
};
} // namespace vpi