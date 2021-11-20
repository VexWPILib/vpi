// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#undef __ARM_NEON__
#undef __ARM_NEON

#include "Eigen/Core"
#include "Eigen/QR"
#include "vpi/geometry/Translation2d.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/MecanumDriveWheelSpeeds.h"

namespace vpi {

/**
 * Helper class that converts a chassis velocity (dx, dy, and dtheta components)
 * into individual wheel speeds.
 *
 * The inverse kinematics (converting from a desired chassis velocity to
 * individual wheel speeds) uses the relative locations of the wheels with
 * respect to the center of rotation. The center of rotation for inverse
 * kinematics is also variable. This means that you can set your set your center
 * of rotation in a corner of the robot to perform special evasion maneuvers.
 *
 * Forward kinematics (converting an array of wheel speeds into the overall
 * chassis motion) is performs the exact opposite of what inverse kinematics
 * does. Since this is an overdetermined system (more equations than variables),
 * we use a least-squares approximation.
 *
 * The inverse kinematics: [wheelSpeeds] = [wheelLocations] * [chassisSpeeds]
 * We take the Moore-Penrose pseudoinverse of [wheelLocations] and then
 * multiply by [wheelSpeeds] to get our chassis speeds.
 *
 * Forward kinematics is also used for odometry -- determining the position of
 * the robot on the field using encoders and a gyro.
 */
class MecanumDriveKinematics {
 public:
  /**
   * Constructs a mecanum drive kinematics object.
   *
   * TODO - First verify this works on a rectangular drive and then move
   * on to more exotic configurations.
   *
   * @param frontLeftWheel The location of the front-left wheel relative to the
   *                       physical center of the robot.
   * @param frontRightWheel The location of the front-right wheel relative to
   *                        the physical center of the robot.
   * @param rearLeftWheel The location of the rear-left wheel relative to the
   *                      physical center of the robot.
   * @param rearRightWheel The location of the rear-right wheel relative to the
   *                       physical center of the robot.
   */
  explicit MecanumDriveKinematics(Translation2d frontLeftWheel,
                                  Translation2d frontRightWheel,
                                  Translation2d rearLeftWheel,
                                  Translation2d rearRightWheel, 
                                  QLength wheelDiameter,
                                  double gearRatio)
      : m_frontLeftWheel{frontLeftWheel},
        m_frontRightWheel{frontRightWheel},
        m_rearLeftWheel{rearLeftWheel},
        m_rearRightWheel{rearRightWheel},
        m_wheelDiameter(wheelDiameter),
        m_gearRatio(gearRatio) {
    SetInverseKinematics(frontLeftWheel, frontRightWheel, rearLeftWheel,
                         rearRightWheel);
    m_trackWidth = frontLeftWheel.X() - frontRightWheel.X();
    m_forwardKinematics = m_inverseKinematics.householderQr();
  }

  MecanumDriveKinematics(const MecanumDriveKinematics&) = default;

  /**
   * Performs inverse kinematics to return the wheel speeds from a desired
   * chassis velocity. This method is often used to convert joystick values into
   * wheel speeds.
   *
   * This function also supports variable centers of rotation. During normal
   * operations, the center of rotation is usually the same as the physical
   * center of the robot; therefore, the argument is defaulted to that use case.
   * However, if you wish to change the center of rotation for evasive
   * maneuvers, vision alignment, or for any other use case, you can do so.
   *
   * @param chassisSpeeds The desired chassis speed.
   * @param centerOfRotation The center of rotation. For example, if you set the
   *                         center of rotation at one corner of the robot and
   *                         provide a chassis speed that only has a dtheta
   *                         component, the robot will rotate around that
   *                         corner.
   *
   * @return The wheel speeds. Use caution because they are not normalized.
   *         Sometimes, a user input may cause one of the wheel speeds to go
   *         above the attainable max velocity. Use the
   *         MecanumDriveWheelSpeeds::Normalize() function to rectify this
   *         issue. In addition, you can leverage the power of C++17 to directly
   *         assign the wheel speeds to variables:
   *
   * @code{.cpp}
   * auto [fl, fr, bl, br] = kinematics.ToWheelSpeeds(chassisSpeeds);
   * @endcode
   */
  MecanumDriveWheelSpeeds ToWheelSpeeds(
      const ChassisSpeeds& chassisSpeeds,
      const Translation2d& centerOfRotation = Translation2d()) const;

  /**
   * Performs forward kinematics to return the resulting chassis state from the
   * given wheel speeds. This method is often used for odometry -- determining
   * the robot's position on the field using data from the real-world speed of
   * each wheel on the robot.
   *
   * @param wheelSpeeds The current mecanum drive wheel speeds.
   *
   * @return The resulting chassis speed.
   */
  ChassisSpeeds ToChassisSpeeds(
      const MecanumDriveWheelSpeeds& wheelSpeeds) const;

    /**
     * TODO - This may need to be modified by cos(45_degrees) given that the
     * wheels are effectively offset at 45 degree angle to the robot's
     * "forward" direction.
     *
     * Given the robot wants to turn an amount, how much different should the
     * left side move than the right.  To pivot in place, one would have the
     * both the left and right sides move this distanc in opposite directions.
     *
     * To turn the robot 360 degrees, the wheels would need to turn the circumference
     * of the circle made by the trackwidth of the robot:
     *    360 degree turn = trackWidth * pi
     *
     * @param The robot's angle change
     * @return differenceLeftToRightWheelDist Distance difference traveled from L to R
     */
    QLength WheelDistanceDifferenceForTurn(QAngle angleToTurn) {
      return ((angleToTurn.convert(degree) / 360.0) * m_trackWidth * 1_pi);
    }

    /**
     * TODO - This may need to be modified by cos(45_degrees) given that the
     * wheels are effectively offset at 45 degree angle to the robot's
     * "forward" direction.
     */
    QAngle LinearDistanceToMotorRotations(QLength d) {
      return UnitUtils::convertLinearToRotational(d, m_wheelDiameter, m_gearRatio);
    }

 private:
  mutable Eigen::Matrix<double, 4, 3> m_inverseKinematics;
  Eigen::HouseholderQR<Eigen::Matrix<double, 4, 3>> m_forwardKinematics;
  Translation2d m_frontLeftWheel;
  Translation2d m_frontRightWheel;
  Translation2d m_rearLeftWheel;
  Translation2d m_rearRightWheel;
  QLength m_wheelDiameter;
  QLength m_trackWidth;
  double m_gearRatio;

  mutable Translation2d m_previousCoR;

  /**
   * Construct inverse kinematics matrix from wheel locations.
   *
   * @param fl The location of the front-left wheel relative to the physical
   *           center of the robot.
   * @param fr The location of the front-right wheel relative to the physical
   *           center of the robot.
   * @param rl The location of the rear-left wheel relative to the physical
   *           center of the robot.
   * @param rr The location of the rear-right wheel relative to the physical
   *           center of the robot.
   */
  void SetInverseKinematics(Translation2d fl, Translation2d fr,
                            Translation2d rl, Translation2d rr) const;
};

}  // namespace vpi