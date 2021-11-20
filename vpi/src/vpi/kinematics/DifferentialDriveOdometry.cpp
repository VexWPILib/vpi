// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/kinematics/DifferentialDriveOdometry.h"

#include "vex.h"

using namespace vpi;

DifferentialDriveOdometry::DifferentialDriveOdometry(const QLength& trackWidth,
    const QLength& wheelDiameter, const Rotation2d& gyroAngle, const double& gearRatio,
    const Pose2d& initialPose)
    : m_pose(initialPose), m_kinematics(trackWidth, wheelDiameter, gearRatio) {
  m_trackWidth = trackWidth;
  m_wheelDiameter = wheelDiameter;
  m_previousAngle = m_pose.Rotation();
  m_gyroOffset = m_pose.Rotation() - gyroAngle;
}

const Pose2d& DifferentialDriveOdometry::Update(const Rotation2d& gyroAngle,
                                                QLength leftDistance,
                                                QLength rightDistance,
                                                QLength strafeDistance) {
  QLength deltaLeftDistance = leftDistance - m_prevLeftDistance;
  QLength deltaRightDistance = rightDistance - m_prevRightDistance;
  QLength deltaStrafeDistance = strafeDistance - m_prevStrafeDistance;

  m_prevLeftDistance = leftDistance;
  m_prevRightDistance = rightDistance;
  m_prevStrafeDistance = strafeDistance;

  QLength averageDeltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0;
  Rotation2d a = gyroAngle + m_gyroOffset;

  Twist2d t;
  t.dx = averageDeltaDistance;
  t.dy = deltaStrafeDistance;
  t.dtheta = a.ToAngle() - m_previousAngle.ToAngle();
  auto newPose = m_pose.Exp(t);

  m_previousAngle = a;
  m_pose = {newPose.Translation(), a};

  return m_pose;
}

const Pose2d& DifferentialDriveOdometry::Update(QLength leftDistance,
                                                QLength rightDistance,
                                                QLength strafeDistance) {
  QLength deltaLeftDistance = leftDistance - m_prevLeftDistance;
  QLength deltaRightDistance = rightDistance - m_prevRightDistance;
  QLength deltaStrafeDistance = strafeDistance - m_prevStrafeDistance;

  m_prevLeftDistance = leftDistance;
  m_prevRightDistance = rightDistance;
  m_prevStrafeDistance = strafeDistance;

  QLength averageDeltaDistance = (deltaLeftDistance + deltaRightDistance) / 2.0;
  QAngle dTheta = m_kinematics.HeadingChange(deltaLeftDistance - deltaRightDistance);
  Rotation2d a(dTheta);

  Twist2d t;
  t.dx = averageDeltaDistance;
  t.dy = deltaStrafeDistance;
  t.dtheta = a.ToAngle();
  auto newPose = m_pose.Exp(t);

  m_previousAngle = a;
  m_pose = {newPose.Translation(), newPose.Rotation()};

  return m_pose;
}