// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/trajectory/TrajectoryConstraint.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QSpeed.h"

namespace vpi {
/**
 * Represents a constraint that enforces a max velocity. This can be composed
 * with the EllipticalRegionConstraint or RectangularRegionConstraint to enforce
 * a max velocity within a region.
 */
class MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  /**
   * Constructs a new MaxVelocityConstraint.
   *
   * @param maxVelocity The max velocity.
   */
  explicit MaxVelocityConstraint(QSpeed maxVelocity);

  QSpeed MaxVelocity(
      const Pose2d& pose, QCurvature curvature,
      QSpeed velocity) const override;

  MinMax MinMaxAcceleration(const Pose2d& pose, QCurvature curvature,
                            QSpeed speed) const override;

 private:
  QSpeed m_maxVelocity;
};
}  // namespace vpi