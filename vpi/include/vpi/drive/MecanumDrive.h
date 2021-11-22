// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <string>

#include "vpi/drive/RobotDriveBase.h"
#include "vpi/kinematics/MecanumDriveWheelSpeeds.h"
#include "vpi/kinematics/MecanumDriveKinematics.h"
#include "vpi/units/QLength.h"
#include "vpi/units/UnitUtils.h"
#include "vex.h"

namespace vpi {

/**
 * 
 * TODO - Complete this class once the Mecanum kinematics classes are complete
 * 
 * A class for driving Mecanum drive platforms.
 *
 * Mecanum drives are rectangular with one wheel on each corner. Each wheel has
 * rollers toed in 45 degrees toward the front or back. When looking at the
 * wheels from the top, the roller axles should form an X across the robot.
 *
 * Drive base diagram:
 * <pre>
 * \\_______/
 * \\ |   | /
 *   |   |
 * /_|___|_\\
 * /       \\
 * </pre>
 *
 * Each Drive() function provides different inverse kinematic relations for a
 * Mecanum drive robot. Motor outputs for the right side are negated, so motor
 * direction inversion by the user is usually unnecessary.
 *
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * The positive X axis points ahead, the positive Y axis points to the right,
 * and the positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 *
 * Inputs smaller then 0.02 will be set to 0, and larger values will be scaled
 * so that the full range is still used. This deadband value can be changed
 * with SetDeadband().
 *
 */
class MecanumDrive : public RobotDriveBase {
 public:
  struct WheelSpeeds {
    double frontLeft = 0.0;
    double frontRight = 0.0;
    double rearLeft = 0.0;
    double rearRight = 0.0;
  };

  /**
   * Construct a MecanumDrive.
   *
   * If a motor needs to be inverted, do so before passing it in.
   */
  MecanumDrive(QLength wheelTrack, QLength wheelBase, QLength wheelDiameter,
               vex::motor& frontLeftMotor, 
               vex::motor& rearLeftMotor,
               vex::motor& frontRightMotor,
               vex::motor& rearRightMotor, double gearRatio = 1.0) :
               RobotDriveBase(wheelTrack,wheelDiameter, gearRatio),
               m_frontLeftMotor(&frontLeftMotor),
               m_rearLeftMotor(&rearLeftMotor),
               m_frontRightMotor(&frontRightMotor),
               m_rearRightMotor(&rearRightMotor),
               m_kinematics({wheelTrack / 2.0, wheelBase / 2.0},
                            {wheelTrack / 2.0, wheelBase / 2.0},
                            {wheelTrack / 2.0, wheelBase / 2.0},
                            {wheelTrack / 2.0, wheelBase / 2.0},
                            wheelDiameter, gearRatio )
  {  }

  ~MecanumDrive() = default;

  MecanumDrive(MecanumDrive&&) = default;
  MecanumDrive& operator=(MecanumDrive&&) = default;

  /**
   * Drive method for Mecanum platform.
   *
   * Angles are measured clockwise from the positive X axis. The robot's speed
   * is independent from its angle or rotation rate.
   *
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is
   *                  positive.
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                  positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around
   *                  the Z axis. Use this to implement field-oriented controls.
   */
  void DriveCartesian(double ySpeed, double xSpeed, double zRotation,
                      double gyroAngle = 0.0);

  /**
   * Cartesian inverse kinematics for Mecanum platform.
   *
   * Angles are measured clockwise from the positive X axis. The robot's speed
   * is independent from its angle or rotation rate.
   *
   * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is
   *                  positive.
   * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                  positive.
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   * @param gyroAngle The current angle reading from the gyro in degrees around
   *                  the Z axis. Use this to implement field-oriented controls.
   */
  static WheelSpeeds DriveCartesianIK(double ySpeed, double xSpeed,
                                      double zRotation, double gyroAngle = 0.0);

  /**
   * Drive method for Mecanum platform.
   *
   * Angles are measured clockwise from the positive X axis. The robot's speed
   * is independent from its angle or rotation rate.
   *
   * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is
   *                  positive.
   * @param angle     The angle around the Z axis at which the robot drives in
   *                  degrees [-180..180].
   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                  Clockwise is positive.
   */
  void DrivePolar(double magnitude, double angle, double zRotation);

  /**
   * Directly provide the wheel speeds
   *
   * @param ddws     The wheel speeds
   */
  void DirectWheelSpeedDrive(MecanumDriveWheelSpeeds ddws);

  void Stop() override;

  void TurnAngle(QAngle target, 
                    QAngularSpeed movementSpeed,
                    bool waitForCompletion=true) override;

  void TurnToPoint(Pose2d currentPose, Point2d target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion=true) override;

  void DriveDistance(QLength target, 
                    QSpeed movementSpeed,
                    bool waitForCompletion=true) override;

  void DriveToPoint(Pose2d currentPose, Point2d target, 
                    QSpeed movementSpeed,
                    bool waitForCompletion=true) override;

  bool IsMoving() override;
  
  void SetBrakeType(vex::brakeType brake) override {
    RobotDriveBase::SetBrakeType(brake);
    m_frontLeftMotor->setStopping(brake);
    m_rearLeftMotor->setStopping(brake);
    m_frontRightMotor->setStopping(brake);
    m_rearRightMotor->setStopping(brake);
  }

 private:
  vex::motor* m_frontLeftMotor;
  vex::motor* m_rearLeftMotor;
  vex::motor* m_frontRightMotor;
  vex::motor* m_rearRightMotor;

 protected:

  MecanumDriveKinematics m_kinematics; // TODO Push down to Base class?

  /**
   * Sets the motor speed based on percent (range from -1 to 1)
   *
   * @param leftFrontSpeed    The robot left side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   * @param rightFrontSpeed   The robot right side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   * @param leftRearSpeed    The robot left side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   * @param rightRearSpeed   The robot right side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   */
  void SetMotorSpeedPercent(double leftFrontSpeed, double rightFrontSpeed,
                            double leftRearSpeed, double rightRearSpeed);
  void SetMotorSpeed(QSpeed leftFrontSpeed, QSpeed rightFrontSpeed,
                      QSpeed leftRearSpeed, QSpeed rightRearSpeed);

  void TurnMotorsForDistanceAtSpeed(QLength lfd, QLength rfd, 
                                  QLength lrd, QLength rrd, 
                                  QSpeed lfs, QSpeed rfs, 
                                  QSpeed lrs, QSpeed rrs, 
                                  bool waitForCompletion);

};

}  // namespace vpi