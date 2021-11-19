// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/kinematics/MecanumDriveOdometry.h"

using namespace vpi;

MecanumDriveOdometry::MecanumDriveOdometry(MecanumDriveKinematics kinematics,
                                           const Rotation2d& gyroAngle,
                                           const Pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {
  m_previousAngle = m_pose.Rotation();
  m_gyroOffset = m_pose.Rotation() - gyroAngle;
}

const Pose2d& MecanumDriveOdometry::UpdateWithTime(
    QTime currentTime, const Rotation2d& gyroAngle,
    MecanumDriveWheelSpeeds wheelSpeeds) {
  QTime deltaTime =
      (m_previousTime >= 0_s) ? currentTime - m_previousTime : 0_s;
  m_previousTime = currentTime;

  auto angle = gyroAngle + m_gyroOffset;

  //auto [dx, dy, dtheta] = m_kinematics.ToChassisSpeeds(wheelSpeeds);
  //static_cast<void>(dtheta);
  ChassisSpeeds cs = m_kinematics.ToChassisSpeeds(wheelSpeeds);

  auto newPose = m_pose.Exp(
      {cs.vx * deltaTime, cs.vy * deltaTime, (angle - m_previousAngle).ToAngle()});

  m_previousAngle = angle;
  m_pose = {newPose.Translation(), angle};

  return m_pose;
}