// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/trajectory/MaxVelocityConstraint.h"

using namespace vpi;

MaxVelocityConstraint::MaxVelocityConstraint(
    QSpeed maxVelocity)
    : m_maxVelocity(fabs(maxVelocity.convert(mps)) * mps) {}

QSpeed MaxVelocityConstraint::MaxVelocity(
    const Pose2d& pose, QCurvature curvature,
    QSpeed velocity) const {
  return m_maxVelocity;
}

TrajectoryConstraint::MinMax MaxVelocityConstraint::MinMaxAcceleration(
    const Pose2d& pose, QCurvature curvature,
    QSpeed speed) const {
  return {};
}