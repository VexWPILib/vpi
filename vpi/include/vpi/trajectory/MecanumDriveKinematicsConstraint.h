// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include "vpi/kinematics/MecanumDriveKinematics.h"
#include "vpi/trajectory/TrajectoryConstraint.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QSpeed.h"

namespace vpi {
/**
 * A class that enforces constraints on the mecanum drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for wheels of the drivetrain stay below a certain
 * limit.
 */
class MecanumDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  MecanumDriveKinematicsConstraint(const MecanumDriveKinematics& kinematics,
                                   QSpeed maxSpeed);

  QSpeed MaxVelocity(
      const Pose2d& pose, QCurvature curvature,
      QSpeed velocity) const override;

  MinMax MinMaxAcceleration(const Pose2d& pose, QCurvature curvature,
                            QSpeed speed) const override;

 private:
  const MecanumDriveKinematics& m_kinematics;
  QSpeed m_maxSpeed;
};
}  // namespace vpi