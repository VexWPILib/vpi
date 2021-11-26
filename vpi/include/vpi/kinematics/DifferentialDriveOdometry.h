// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/kinematics/DifferentialDriveKinematics.h"
#include "vpi/geometry/Pose2d.h"
#include "vpi/geometry/VexGpsPose2d.h"
#include "vpi/units/QLength.h"

namespace vpi {
/**
 * Class for differential drive odometry. Odometry allows you to track the
 * robot's position on the field over the course of a match using readings from
 * 2 encoders and a gyroscope.
 *
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following. Furthermore, odometry can be used for latency compensation
 * when using computer-vision systems.
 *
 * It is important that you reset your encoders to zero before using this class.
 * Any subsequent pose resets also require the encoders to be reset to zero.
 *
 * An Odometry class does not have motors, but could have rotation
 * sensors, or be fed values from rotation sensors, gyros, or vision sensors.
 *
 * To differentiate terms:
 * * Drives has motors and powers the robot
 * * Kinematics deal with the robot in motion, and track position
 * * Chassis is a fusion of a Drive and Kinematic
 *
 * Generally, users should program against a Chassis
 *
 * See: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html
 */
class DifferentialDriveOdometry {
 public:
  /**
   * Constructs a DifferentialDriveOdometry object.
   *
   * @param trackWidth The trackWidth of the robot
   * @param wheelDiameter The size of the wheels
   * @param gyroAngle The angle reported by the gyroscope.
   * @param gearRatio Any external gearing, defaults to 1.0
   * @param initialPose The starting position of the robot on the field.
   */
  explicit DifferentialDriveOdometry(const QLength& trackWidth,
                                     const QLength& wheelDiameter,
                                     const Rotation2d& gyroAngle,
                                     const double& gearRatio = 1.0,
                                     const Pose2d& initialPose = Pose2d({0_m, 0_m}, 0_deg));

  /**
   * Resets the robot's position on the field.
   *
   * You NEED to reset your encoders (to zero) when calling this method.
   *
   * The gyroscope angle does not need to be reset here on the user's robot
   * code. The library automatically takes care of offsetting the gyro angle.
   *
   * @param pose The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  virtual void ResetPosition(const VexGpsPose2d& pose, const Rotation2d& gyroAngle) {
    m_pose = pose;
    m_previousAngle = m_pose.Rotation();
    m_gyroOffset = Rotation2d(pose.Theta()) - gyroAngle;

    m_prevLeftDistance = 0_m;
    m_prevRightDistance = 0_m;
    m_prevStrafeDistance = 0_m;
  }

  /**
   * Resets the robot's position on the field.
   *
   * You NEED to reset your encoders (to zero) when calling this method.
   *
   * @param pose The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  virtual void ResetPosition(const VexGpsPose2d& pose) {
    m_pose = pose;
    m_previousAngle = Rotation2d(pose.Theta());

    m_prevLeftDistance = 0_m;
    m_prevRightDistance = 0_m;
    m_prevStrafeDistance = 0_m;
  }

  /**
   * Returns the position of the robot on the field.
   * @return The pose of the robot.
   */
  const Pose2d& GetPose() const { return m_pose; }

  /**
   * Updates the robot position on the field using distance measurements from
   * encoders. This method is more numerically accurate than using velocities to
   * integrate the pose.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param leftDistance The distance traveled by the left encoder.
   * @param rightDistance The distance traveled by the right encoder.
   * @param strafeDistance The distance traveled by the strafe encoder.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(const Rotation2d& gyroAngle, QLength leftDistance,
                       QLength rightDistance, QLength strafeDistnace = 0_m);

  /**
   * Updates the robot position on the field using distance measurements from
   * encoders and should be used on robots without a gyroscope
   *
   * @param leftDistance The distance traveled by the left encoder.
   * @param rightDistance The distance traveled by the right encoder.
   * @param strafeDistance The distance traveled by the strafe encoder.
   *
   * @return The new pose of the robot.
   */
  const Pose2d& Update(QLength leftDistance, QLength rightDistance,
                       QLength strafeDistnace = 0_m);

 protected:
  QLength m_trackWidth;
  QLength m_wheelDiameter;
  Pose2d m_pose;
  DifferentialDriveKinematics m_kinematics;
  
 private:
  Rotation2d m_gyroOffset;
  Rotation2d m_previousAngle;

  QLength m_prevLeftDistance = 0_m;
  QLength m_prevRightDistance = 0_m;
  QLength m_prevStrafeDistance = 0_m;
};
}  // namespace vpi
