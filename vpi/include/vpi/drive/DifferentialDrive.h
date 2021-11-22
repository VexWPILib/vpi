// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/drive/RobotDriveBase.h"
#include "vpi/kinematics/DifferentialDriveKinematics.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/units/QLength.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {

/**
 * A class for driving differential drive/skid-steer drive platforms such as
 * the Kit of Parts drive base, "tank drive", or West Coast Drive.
 *
 * These drive bases typically have drop-center / skid-steer with two or more
 * wheels per side (e.g., 4WD or 6WD). This class takes a MotorController per
 * side. For four and six motor drivetrains, construct and pass in
 * MotorControllerGroup instances as follows.
 *
 * Four motor drivetrain:
 * @code{.cpp}
 * class Chassis {
 *  public:
 *   vex::motor Motor1 = motor(PORT1, ratio18_1, false);
 *   vex::motor Motor2 = motor(PORT2, ratio18_1, false);
 *   vex::motor_group m_left{Motor1, Motor2};
 *
 *   vex::motor Motor3 = motor(PORT3, ratio18_1, false);
 *   vex::motor Motor4 = motor(PORT4, ratio18_1, false);
 *   vex::motor_group m_right{Motor3, Motor4};
 *
 *   vpi::DifferentialDrive m_drive{m_left, m_right};
 * };
 * @endcode
 *
 * A differential drive robot has left and right wheels separated by an
 * arbitrary width.
 *
 * Drive base diagram:
 * <pre>
 * |_______|
 * | |   | |
 *   |   |
 * |_|___|_|
 * |       |
 * </pre>
 *
 * Each Drive() function provides different inverse kinematic relations for a
 * differential drive robot. Motor outputs for the right side are negated, so
 * motor direction inversion by the user is usually unnecessary.
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
 */
class DifferentialDrive : public RobotDriveBase {
 public:
  struct WheelSpeeds {
    double left = 0.0;
    double right = 0.0;
  };

  /**
   * Construct a DifferentialDrive.
   *
   * A motor_group can contain one or more motors. If a motor
   * needs to be inverted, do so before passing it in.
   */
  DifferentialDrive(QLength wheelTrack, QLength wheelDiameter,
                    vex::motor_group& leftMotorGroup, 
                    vex::motor_group& rightMotorGroup,
                    double gearRatio = 1.0)
        : RobotDriveBase(wheelTrack, wheelDiameter, gearRatio), 
          m_leftMotor(&leftMotorGroup), m_rightMotor(&rightMotorGroup),
          m_kinematics(wheelTrack, wheelDiameter, gearRatio)
  {   }

  ~DifferentialDrive() = default;

  DifferentialDrive(DifferentialDrive&&) = default;
  DifferentialDrive& operator=(DifferentialDrive&&) = default;

  /**
   * Arcade drive method for differential drive platform.
   *
   * Note: Some drivers may prefer inverted rotation controls. This can be done
   * by negating the value passed for rotation.
   *
   * @param xSpeed        The speed at which the robot should drive along the X
   *                      axis [-1.0..1.0]. Forward is positive.
   * @param zRotation     The rotation rate of the robot around the Z axis
   *                      [-1.0..1.0]. Clockwise is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  void ArcadeDrive(double xSpeed, double zRotation, bool squareInputs = true);

  /**
   * Arcade drive inverse kinematics for differential drive platform.
   *
   * Note: Some drivers may prefer inverted rotation controls. This can be done
   * by negating the value passed for rotation.
   *
   * @param xSpeed       The speed at which the robot should drive along the X
   *                     axis [-1.0..1.0]. Forward is positive.
   * @param zRotation    The rotation rate of the robot around the Z axis
   *                     [-1.0..1.0]. Clockwise is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  static WheelSpeeds ArcadeDriveIK(double xSpeed, double zRotation,
                                   bool squareInputs = true);

  /**
   * Tank drive method for differential drive platform.
   *
   * @param leftSpeed     The robot left side's speed along the X axis
   *                      [-1.0..1.0]. Forward is positive.
   * @param rightSpeed    The robot right side's speed along the X axis
   *                      [-1.0..1.0]. Forward is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  void TankDrive(double leftSpeed, double rightSpeed, bool squareInputs = true);

  /**
   * Tank drive inverse kinematics for differential drive platform.
   *
   * @param leftSpeed    The robot left side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   * @param rightSpeed   The robot right side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  static WheelSpeeds TankDriveIK(double leftSpeed, double rightSpeed,
                                 bool squareInputs = true);

  /**
   * Directly provide the wheel speeds
   *
   * @param ddws     The wheel speeds
   */
  void DirectWheelSpeedDrive(DifferentialDriveWheelSpeeds ddws);

  /**
   * Directly provide the wheel speeds
   *
   * @param cs     The chassis speeds
   */
  void DriveChassisSpeeds(ChassisSpeeds cs) override {
    DirectWheelSpeedDrive(m_kinematics.ToWheelSpeeds(cs));
  }

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

  void SetBrakeType(vex::brakeType brake) override {
    RobotDriveBase::SetBrakeType(brake);
    m_leftMotor->setStopping(brake);
    m_rightMotor->setStopping(brake);
  }

  bool IsMoving() override;

 protected:
  vex::motor_group* m_leftMotor;
  vex::motor_group* m_rightMotor;
  DifferentialDriveKinematics m_kinematics;

  /**
   * Sets the motor speed based on percent (range from -1 to 1)
   *
   * @param leftSpeed    The robot left side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   * @param rightSpeed   The robot right side's speed along the X axis
   *                     [-1.0..1.0]. Forward is positive.
   */
  void SetMotorSpeedPercent(double leftSpeed, double rightSpeed);
  void SetMotorSpeed(QSpeed leftSpeed, QSpeed rightSpeed);

  void TurnMotorsForDistanceAtSpeed(QLength ld, QLength rd, 
                                  QSpeed ls, QSpeed rs, 
                                  bool waitForCompletion);
};

}  // namespace vpi
