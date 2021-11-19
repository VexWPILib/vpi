// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/trajectory/DifferentialDriveKinematicsConstraint.h"

using namespace vpi;

DifferentialDriveKinematicsConstraint::DifferentialDriveKinematicsConstraint(
    const DifferentialDriveKinematics& kinematics,
    QSpeed maxSpeed)
    : m_kinematics(kinematics), m_maxSpeed(maxSpeed) {}

QSpeed DifferentialDriveKinematicsConstraint::MaxVelocity(
    const Pose2d& pose, QCurvature curvature,
    QSpeed velocity) const {
  auto wheelSpeeds =
      m_kinematics.ToWheelSpeeds({velocity, 0_mps, velocity * curvature});
  wheelSpeeds.Normalize(m_maxSpeed);

  return m_kinematics.ToChassisSpeeds(wheelSpeeds).vx;
}

TrajectoryConstraint::MinMax
DifferentialDriveKinematicsConstraint::MinMaxAcceleration(
    const Pose2d& pose, QCurvature curvature,
    QSpeed speed) const {
  return {};
}