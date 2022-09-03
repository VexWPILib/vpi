// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/follower/RamseteController.h"

#include <cmath>
#include <math.h>

using namespace vpi;

/**
 * Returns sin(x) / x.
 *
 * @param x Value of which to take sinc(x).
 */
static double Sinc(double x) {
  if (std::abs(x) < 1e-9) {
    return 1.0 - 1.0 / 6.0 * x * x;
  } else {
    return std::sin(x) / x;
  }
}

RamseteController::RamseteController(double b, double zeta)
    : m_b{b}, m_zeta{zeta} {}

bool RamseteController::AtReference() const {
  const auto& eTranslate = m_poseError.Translation();
  const auto& eRotate = m_poseError.Rotation();
  const auto& tolTranslate = m_poseTolerance.Translation();
  const auto& tolRotate = m_poseTolerance.Rotation();
  return std::fabs(eTranslate.X().convert(meter)) < tolTranslate.X().convert(meter) &&
         std::fabs(eTranslate.Y().convert(meter)) < tolTranslate.Y().convert(meter) &&
         std::fabs(eRotate.ToAngle().convert(radian)) < tolRotate.ToAngle().convert(radian);
}

void RamseteController::SetTolerance(const Pose2d& poseTolerance) {
  m_poseTolerance = poseTolerance;
}

ChassisSpeeds RamseteController::Calculate(
    const Pose2d& currentPose, const Pose2d& poseRef,
    QSpeed linearVelocityRef,
    QAngularSpeed angularVelocityRef) {
  if (!m_enabled) {
    return ChassisSpeeds{linearVelocityRef, 0_mps, angularVelocityRef};
  }

  m_poseError = poseRef.RelativeTo(currentPose);

  double eX = m_poseError.X().convert(meter);
  double eY = m_poseError.Y().convert(meter);
  double eTheta = m_poseError.Rotation().ToAngle().convert(radian);
  double vRef = linearVelocityRef.convert(mps);
  double omegaRef = angularVelocityRef.convert(radps);

  double k =
      2.0 * m_zeta * std::sqrt(std::pow(omegaRef, 2) + m_b * std::pow(vRef, 2));

  QSpeed v = mps * (vRef * m_poseError.Rotation().Cos() + k * eX);
  QAngularSpeed omega = (omegaRef + k * eTheta +
                                    m_b * vRef * Sinc(eTheta) * eY) * radps;
  return ChassisSpeeds{v, 0_mps, omega};
}

ChassisSpeeds RamseteController::Calculate(
    const Pose2d& currentPose, const Trajectory::State& desiredState) {
  return Calculate(currentPose, desiredState.pose, desiredState.velocity,
                   desiredState.velocity * desiredState.curvature);
}

void RamseteController::SetEnabled(bool enabled) {
  m_enabled = enabled;
}