// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/trajectory/CentripetalAccelerationConstraint.h"

using namespace vpi;

CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(
    QAcceleration maxCentripetalAcceleration)
    : m_maxCentripetalAcceleration(maxCentripetalAcceleration) {}

QSpeed CentripetalAccelerationConstraint::MaxVelocity(
    const Pose2d& pose, QCurvature curvature,
    QSpeed velocity) const {
  // ac = v^2 / r
  // k (curvature) = 1 / r

  // therefore, ac = v^2 * k
  // ac / k = v^2
  // v = std::sqrt(ac / k)

  // We have to multiply by 1_rad here to get the units to cancel out nicely.
  // The units library defines a unit for radians although it is technically
  // unitless.
  return std::sqrt(m_maxCentripetalAcceleration.convert(mps2) /
                           fabs(curvature.convert(radpermeter))) * mps;
}

TrajectoryConstraint::MinMax
CentripetalAccelerationConstraint::MinMaxAcceleration(
    const Pose2d& pose, QCurvature curvature,
    QSpeed speed) const {
  // The acceleration of the robot has no impact on the centripetal acceleration
  // of the robot.
  return {};
}