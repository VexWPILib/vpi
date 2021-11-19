// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/geometry/Point2d.h"
#include "vpi/geometry/Pose2d.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/units/QAngle.h"
#include "vpi/units/QLength.h"

namespace vpi {

/**
 * Common base class for drive platforms.
 *
 * To differentiate terms:
 * * Drives has motors and powers the robot
 * * Kinematics deal with the robot in motion, and track position
 * * Chassis is a fusion of a Drive and Kinematic
 *
 * Generally, users should program against a Chassis
 */
class RobotDriveBase {
 public:
  /**
   * The location of a motor on the robot for the purpose of driving.
   */
  enum MotorType {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
    kLeft = 0,
    kRight = 1,
    kBack = 2
  };

  RobotDriveBase(QLength wheelTrack, QLength wheelDiameter, double gearRatio = 1.0) :
    m_wheelTrack(wheelTrack), m_wheelDiameter(wheelDiameter)
  {
    m_gearRatio = gearRatio;
  }
  
  ~RobotDriveBase() = default;

  RobotDriveBase(RobotDriveBase&&) = default;
  RobotDriveBase& operator=(RobotDriveBase&&) = default;

  /**
   * Sets the deadband applied to the drive inputs (e.g., joystick values).
   *
   * The default value is 0.02. Inputs smaller than the deadband are set to 0.0
   * while inputs larger than the deadband are scaled from 0.0 to 1.0.
   *
   * @param deadband The deadband to set.
   */
  void SetDeadband(double deadband) {
    m_deadband = deadband;
  }

  /**
   * Sets the brake type for the drive motors to use.
   *
   * @param breakType The brake type.
   */
  virtual void SetBrakeType(vex::brakeType brake) {
    m_brakeType = brake;
  }

  /**
   * Sets the wheel diameter.
   *
   * @param wheelDiameter The diameter of the wheels.
   */
  void SetWheelDiameter(QLength wheelDiameter) {
    m_wheelDiameter = wheelDiameter;
  }

  /**
   * Sets the gear ratio from the motors to the wheels
   *
   * @param gearRatio The gear ratio.
   */
  void SetGearRatio(double gearRatio) {
    m_gearRatio = gearRatio;
  }

  virtual void Stop();

  /**
   * Turns the robot.
   *
   * @param target The amount to turn
   *
   * @param turnSpeed How fast to turn
   *
   * @param waitForCompletion Block until movement completes
   */
  virtual void TurnAngle(QAngle target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion=true) = 0;

  /**
   * Drives the robot the desired distance
   *
   * @param target The target distance to move
   *
   * @param movementSpeed The linear movement speed
   *
   * @param waitForCompletion Block until movement completes
   */
  virtual void DriveDistance(QLength target, 
                            QSpeed movementSpeed,
                            bool waitForCompletion=true) = 0;

  /**
   * Turns the robot to face the desired target point
   *
   * @param currentPosition The robot's current pose
   *                        both the position and the
   *                        heading
   *
   * @param target The target to head towards
   *
   * @param turnSpeed How fast to turn
   *
   * @param waitForCompletion Block until movement completes
   */
  virtual void TurnToPoint(Pose2d currentPose, Point2d target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion=true) = 0;

  /**
   * Turns the robot to face the desired target point
   * and then drives the distance to it.
   *
   * @param currentPosition The robot's current pose
   *                        both the position and the
   *                        heading
   *
   * @param target The target to head towards
   *
   * @param movementSpeed The linear speed to use; will
   *                      use a corresponding value for the turn
   *
   * @param waitForCompletion Block until movement completes
   */
  virtual void DriveToPoint(Pose2d currentPose, Point2d target, 
                          QSpeed movementSpeed,
                          bool waitForCompletion=true) = 0;

  virtual void DriveChassisSpeeds(ChassisSpeeds cs) = 0;

  virtual bool IsMoving() = 0;

 protected:
  double m_deadband = 0.02;
  vex::brakeType m_brakeType = vex::brakeType::coast;
  QLength m_wheelTrack = -99 * meter;
  QLength m_wheelDiameter = 4_in;
  double m_gearRatio = 1.0;

  QAngle computeMotorRotationsForTurn(Pose2d currentPose, Point2d target);
  QAngle computeMotorRotationsForDrive(QLength target);
};

}  // namespace vpi
