// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/units/QAngle.h"
#include "vpi/units/QLength.h"

namespace vpi {
/**
 * Helper class that converts a chassis velocity (dx and dtheta components) to
 * left and right wheel velocities for a differential drive.
 *
 * Inverse kinematics converts a desired chassis speed into left and right
 * velocity components whereas forward kinematics converts left and right
 * component velocities into a linear and angular chassis speed.
 *
 * A Kinematic or Odometry class does not have motors, but could have rotation
 * sensors.
 *
 * To differentiate terms:
 * * Drives has motors and powers the robot
 * * Kinematics deal with the robot in motion, and track position
 * * Chassis is a fusion of a Drive and Kinematic
 *
 * Generally, users should program against a Chassis
 *
 * See: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-kinematics.html
 */
class DifferentialDriveKinematics {
 public:
    /**
     * Constructs a differential drive kinematics object.
     *
     * @param trackWidth The track width of the drivetrain. Theoretically, this is
     * the distance between the left wheels and right wheels. However, the
     * empirical value may be larger than the physical measured value due to
     * scrubbing effects.
     * 
     * @param wheelDiameter The size of wheel, likely close to 3.25 or 4.0 inches
     */
    explicit DifferentialDriveKinematics(QLength trackWidth, 
                                        QLength wheelDiameter,
                                        double gearRatio)
        : m_trackWidth(trackWidth), m_wheelDiameter(wheelDiameter) {
      m_gearRatio = gearRatio;
    }

    /**
     * Returns a chassis speed from left and right component velocities using
     * forward kinematics.
     *
     * @param wheelSpeeds The left and right velocities.
     * @return The chassis speed.
     */
    ChassisSpeeds ToChassisSpeeds(
        const DifferentialDriveWheelSpeeds& wheelSpeeds) const {
      ChassisSpeeds retval;
      retval.vx = (wheelSpeeds.left + wheelSpeeds.right) / 2.0;
      retval.vy = 0_mps;
      retval.omega = (wheelSpeeds.right - wheelSpeeds.left) / m_trackWidth * radian;
      return retval;
    }

    /**
     * Returns left and right component velocities from a chassis speed using
     * inverse kinematics.
     *
     * @param chassisSpeeds The linear and angular (dx and dtheta) components that
     * represent the chassis' speed.
     * @return The left and right velocities.
     */
    DifferentialDriveWheelSpeeds ToWheelSpeeds(
        const ChassisSpeeds& chassisSpeeds) const {
      DifferentialDriveWheelSpeeds retval;
      retval.left = chassisSpeeds.vx - m_trackWidth / 2 * chassisSpeeds.omega / 1_rad;
      retval.right = chassisSpeeds.vx + m_trackWidth / 2 * chassisSpeeds.omega / 1_rad;
      return retval;
    }

    /**
     * Given the difference between how far the left side moved from the right side,
     * return the change in the robot's heading.
     *
     * @param differenceLeftToRightWheelDist Distance difference traveled from L to R
     * @return The robot's angle change
     */
    QAngle HeadingChange(QLength differenceLeftToRightWheelDist) {
      return (differenceLeftToRightWheelDist / m_trackWidth) * radian;
    }

    /**
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

    QAngle LinearDistanceToMotorRotations(QLength d) {
      return UnitUtils::convertLinearToRotational(d, m_wheelDiameter, m_gearRatio);
    }

  protected:
    QLength m_trackWidth;
    QLength m_wheelDiameter;
    double m_gearRatio;
};
}  // namespace vpi