// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/trajectory/MecanumDriveKinematicsConstraint.h"

using namespace vpi;

MecanumDriveKinematicsConstraint::MecanumDriveKinematicsConstraint(
    const MecanumDriveKinematics& kinematics,
    QSpeed maxSpeed)
    : m_kinematics(kinematics), m_maxSpeed(maxSpeed) {}

QSpeed MecanumDriveKinematicsConstraint::MaxVelocity(
    const Pose2d& pose, QCurvature curvature,
    QSpeed velocity) const {
  auto xVelocity = velocity * pose.Rotation().Cos();
  auto yVelocity = velocity * pose.Rotation().Sin();
  auto wheelSpeeds =
      m_kinematics.ToWheelSpeeds({xVelocity, yVelocity, velocity * curvature});
  wheelSpeeds.Normalize(m_maxSpeed);

  auto normSpeeds = m_kinematics.ToChassisSpeeds(wheelSpeeds);

  return hypot(normSpeeds.vx, normSpeeds.vy);
}

TrajectoryConstraint::MinMax
MecanumDriveKinematicsConstraint::MinMaxAcceleration(
    const Pose2d& pose, QCurvature curvature,
    QSpeed speed) const {
  return {};
}