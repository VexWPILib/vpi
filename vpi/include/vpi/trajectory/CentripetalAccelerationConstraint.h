// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/trajectory/TrajectoryConstraint.h"
#include "vpi/units/QAcceleration.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QSpeed.h"

namespace vpi {

/**
 * A constraint on the maximum absolute centripetal acceleration allowed when
 * traversing a trajectory. The centripetal acceleration of a robot is defined
 * as the velocity squared divided by the radius of curvature.
 *
 * Effectively, limiting the maximum centripetal acceleration will cause the
 * robot to slow down around tight turns, making it easier to track trajectories
 * with sharp turns.
 */
class CentripetalAccelerationConstraint
    : public TrajectoryConstraint {
 public:
  explicit CentripetalAccelerationConstraint(
      QAcceleration maxCentripetalAcceleration);

  QSpeed MaxVelocity(
      const Pose2d& pose, QCurvature curvature,
      QSpeed velocity) const override;

  MinMax MinMaxAcceleration(const Pose2d& pose, QCurvature curvature,
                            QSpeed speed) const override;

 private:
  QAcceleration m_maxCentripetalAcceleration;
};
}  // namespace vpi