// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/kinematics/DifferentialDriveKinematics.h"
#include "vpi/trajectory/TrajectoryConstraint.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QSpeed.h"

namespace vpi {
/**
 * A class that enforces constraints on the differential drive kinematics.
 * This can be used to ensure that the trajectory is constructed so that the
 * commanded velocities for both sides of the drivetrain stay below a certain
 * limit.
 */
class DifferentialDriveKinematicsConstraint
    : public TrajectoryConstraint {
 public:
  DifferentialDriveKinematicsConstraint(
      const DifferentialDriveKinematics& kinematics,
      QSpeed maxSpeed);

  QSpeed MaxVelocity(
      const Pose2d& pose, QCurvature curvature,
      QSpeed velocity) const override;

  MinMax MinMaxAcceleration(const Pose2d& pose, QCurvature curvature,
                            QSpeed speed) const override;

 private:
  const DifferentialDriveKinematics& m_kinematics;
  QSpeed m_maxSpeed;
};
}  // namespace vpi