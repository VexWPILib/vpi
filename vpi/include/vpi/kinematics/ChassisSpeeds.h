// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/geometry/Rotation2d.h"
#include "vpi/units/QAngularSpeed.h"
#include "vpi/units/QSpeed.h"

namespace vpi {
/**
 * Represents the speed of a robot chassis. Although this struct contains
 * similar members compared to a Twist2d, they do NOT represent the same thing.
 * Whereas a Twist2d represents a change in pose w.r.t to the robot frame of
 * reference, this ChassisSpeeds struct represents a velocity w.r.t to the robot
 * frame of reference.
 *
 * A strictly non-holonomic drivetrain, such as a differential drive, should
 * never have a dy component because it can never move sideways. Holonomic
 * drivetrains such as swerve and mecanum will often have all three components.
 *
 * See: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html
 */
struct ChassisSpeeds {
  /**
   * Represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
   */
  QSpeed vx = 0_mps;

  /**
   * Represents strafe velocity w.r.t the robot frame of reference. (Right is +)
   *
   * In the original WPI, Left is +
   */
  QSpeed vy = 0_mps;

  /**
   * Represents the angular velocity of the robot frame. (CW is +)
   *
   * In the original WPI, CCW is +
   */
  QAngularSpeed omega = 0 * radps;

  ChassisSpeeds() {
    vx = 0_mps;
    vy = 0_mps;
    omega = 0_radps;
  }

  ChassisSpeeds(QSpeed x, QSpeed y, QAngularSpeed a) {
    vx = x;
    vy = y;
    omega = a;
  }

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds object.
   *
   * @param vx The component of speed in the x direction relative to the field.
   * Positive y is away from Red alliance wall.
   * @param vy The component of speed in the y direction relative to the field.
   * Positive y is to the right when standing behind the Red alliance wall.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The
   * robot's angle is considered to be zero when it is facing directly away from
   * your alliance station wall. Remember that this should be CW positive.
   *
   * @return ChassisSpeeds object representing the speeds in the robot's frame
   * of reference.
   */
  static ChassisSpeeds FromFieldRelativeSpeeds(
      QSpeed vx, QSpeed vy,
      QAngularSpeed omega, const Rotation2d& robotAngle) {
        ChassisSpeeds retval;
        retval.vx = vx * robotAngle.Sin() + vy * robotAngle.Cos();
        retval.vy = -vx * robotAngle.Cos() + vy * robotAngle.Sin();
        retval.omega = omega;
        return retval;
    /* Original WPI:
    return {vx * robotAngle.Cos() + vy * robotAngle.Sin(),
            -vx * robotAngle.Sin() + vy * robotAngle.Cos(), omega};
    */
  }
};
}  // namespace vpi
