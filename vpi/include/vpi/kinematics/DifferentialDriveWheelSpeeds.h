// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/units/QSpeed.h"

namespace vpi {
/**
 * Represents the wheel speeds for a differential drive drivetrain.
 */
struct DifferentialDriveWheelSpeeds {
  /**
   * Speed of the left side of the robot.
   */
  QSpeed left = 0_mps;

  /**
   * Speed of the right side of the robot.
   */
  QSpeed right = 0_mps;

  /**
   * Normalizes the wheel speeds using some max attainable speed. Sometimes,
   * after inverse kinematics, the requested speed from a/several modules may be
   * above the max attainable speed for the driving motor on that module. To fix
   * this issue, one can "normalize" all the wheel speeds to make sure that all
   * requested module speeds are below the absolute threshold, while maintaining
   * the ratio of speeds between modules.
   *
   * @param attainableMaxSpeed The absolute max speed that a wheel can reach.
   */
  void Normalize(QSpeed attainableMaxSpeed);
};
}  // namespace vpi
