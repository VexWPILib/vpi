// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/kinematics/MecanumDriveKinematics.h"

using namespace vpi;

MecanumDriveWheelSpeeds MecanumDriveKinematics::ToWheelSpeeds(
    const ChassisSpeeds& chassisSpeeds,
    const Translation2d& centerOfRotation) const {
  // We have a new center of rotation. We need to compute the matrix again.
  if (centerOfRotation != m_previousCoR) {
    auto fl = m_frontLeftWheel - centerOfRotation;
    auto fr = m_frontRightWheel - centerOfRotation;
    auto rl = m_rearLeftWheel - centerOfRotation;
    auto rr = m_rearRightWheel - centerOfRotation;

    SetInverseKinematics(fl, fr, rl, rr);

    m_previousCoR = centerOfRotation;
  }

  Eigen::Vector3d chassisSpeedsVector{chassisSpeeds.vx.convert(mps),
                                      chassisSpeeds.vy.convert(mps),
                                      chassisSpeeds.omega.convert(radps)};

  Eigen::Vector<double, 4> wheelsVector =
      m_inverseKinematics * chassisSpeedsVector;

  MecanumDriveWheelSpeeds wheelSpeeds;
  wheelSpeeds.frontLeft = wheelsVector(0) * mps;
  wheelSpeeds.frontRight = wheelsVector(1) * mps;
  wheelSpeeds.rearLeft = wheelsVector(2) * mps;
  wheelSpeeds.rearRight = wheelsVector(3) * mps;
  return wheelSpeeds;
}

ChassisSpeeds MecanumDriveKinematics::ToChassisSpeeds(
    const MecanumDriveWheelSpeeds& wheelSpeeds) const {
  Eigen::Vector<double, 4> wheelSpeedsVector{
      wheelSpeeds.frontLeft.convert(mps), wheelSpeeds.frontRight.convert(mps),
      wheelSpeeds.rearLeft.convert(mps), wheelSpeeds.rearRight.convert(mps)};

  Eigen::Vector3d chassisSpeedsVector =
      m_forwardKinematics.solve(wheelSpeedsVector);

  return {chassisSpeedsVector(0) * mps,  // NOLINT
          chassisSpeedsVector(1) * mps,
          chassisSpeedsVector(2) * radps};
}

void MecanumDriveKinematics::SetInverseKinematics(Translation2d fl,
                                                  Translation2d fr,
                                                  Translation2d rl,
                                                  Translation2d rr) const {
  m_inverseKinematics = Eigen::Matrix<double, 4, 3>{
      {1, -1, (-(fl.X().convert(meter) + fl.Y().convert(meter)))},
      {1, 1, (fr.X().convert(meter) - fr.Y().convert(meter))},
      {1, 1, (rl.X().convert(meter) - rl.Y().convert(meter))},
      {1, -1, (-(rr.X().convert(meter) + rr.Y().convert(meter)))}};
}