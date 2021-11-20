// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/drive/DifferentialDrive.h"

#include <algorithm>
#include <cmath>

#include "vpi/utils.h"
#include "vpi/units/UnitUtils.h"

using namespace vpi;

void DifferentialDrive::SetMotorSpeedPercent(double leftSpeed, double rightSpeed)
{
  // TODO - any filtering of the speeds
  m_leftMotor->spin(vex::directionType::fwd, leftSpeed * 100.0, vex::percentUnits::pct);
  m_rightMotor->spin(vex::directionType::fwd, rightSpeed * 100.0, vex::percentUnits::pct);
}

void DifferentialDrive::SetMotorSpeed(QSpeed leftSpeed, QSpeed rightSpeed)
{
  // TODO - any filtering of the speeds
  double leftRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(leftSpeed, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rightRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(rightSpeed, m_wheelDiameter, m_gearRatio).convert(rpm);

  m_leftMotor->spin(vex::directionType::fwd, leftRpm, vex::velocityUnits::rpm);
  m_rightMotor->spin(vex::directionType::fwd, rightRpm, vex::velocityUnits::rpm);
}

void DifferentialDrive::TurnMotorsForDistanceAtSpeed(QLength ld, QLength rd, 
                                QSpeed ls, QSpeed rs, 
                                bool waitForCompletion) {
  double leftDegrees = UnitUtils::convertLinearToRotational(ld, m_wheelDiameter, m_gearRatio).convert(degree);
  double rightDegrees = UnitUtils::convertLinearToRotational(rd, m_wheelDiameter, m_gearRatio).convert(degree);

  // TODO - any filtering of the speeds
  double leftRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(ls, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rightRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(rs, m_wheelDiameter, m_gearRatio).convert(rpm);

  m_leftMotor->spinFor(leftDegrees, vex::rotationUnits::deg, leftRpm, vex::velocityUnits::rpm, false);
  m_rightMotor->spinFor(rightDegrees, vex::rotationUnits::deg, rightRpm, vex::velocityUnits::rpm, false);
  if(waitForCompletion) {
    while(!m_leftMotor->isDone() && !m_rightMotor->isDone()) {
      wait(20, msec);
    }
    Stop();
  }
}

void DifferentialDrive::ArcadeDrive(double xSpeed, double zRotation,
                                    bool squareInputs) {
  xSpeed = VpiUtils::ApplyDeadband(xSpeed, m_deadband);
  zRotation = VpiUtils::ApplyDeadband(zRotation, m_deadband);

  WheelSpeeds ws = ArcadeDriveIK(xSpeed, zRotation, squareInputs);

  SetMotorSpeedPercent(ws.left, ws.right);
}

void DifferentialDrive::TankDrive(double leftSpeed, double rightSpeed,
                                  bool squareInputs) {

  leftSpeed = VpiUtils::ApplyDeadband(leftSpeed, m_deadband);
  rightSpeed = VpiUtils::ApplyDeadband(rightSpeed, m_deadband);

  WheelSpeeds ws = TankDriveIK(leftSpeed, rightSpeed, squareInputs);

  SetMotorSpeedPercent(ws.left, ws.right);
}

DifferentialDrive::WheelSpeeds DifferentialDrive::ArcadeDriveIK(
    double xSpeed, double zRotation, bool squareInputs) {
  xSpeed = VpiUtils::clip(xSpeed, -1.0, 1.0);
  zRotation = VpiUtils::clip(zRotation, -1.0, 1.0);

  // Square the inputs (while preserving the sign) to increase fine control
  // while permitting full power.
  if (squareInputs) {
    xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
    zRotation = std::copysign(zRotation * zRotation, zRotation);
  }

  double leftSpeed;
  double rightSpeed;

  double maxInput =
      std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

  if (xSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (zRotation >= 0.0) {
      leftSpeed = maxInput;
      rightSpeed = xSpeed - zRotation;
    } else {
      leftSpeed = xSpeed + zRotation;
      rightSpeed = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (zRotation >= 0.0) {
      leftSpeed = xSpeed + zRotation;
      rightSpeed = maxInput;
    } else {
      leftSpeed = maxInput;
      rightSpeed = xSpeed - zRotation;
    }
  }

  // Normalize the wheel speeds
  double maxMagnitude = std::max(std::abs(leftSpeed), std::abs(rightSpeed));
  if (maxMagnitude > 1.0) {
    leftSpeed /= maxMagnitude;
    rightSpeed /= maxMagnitude;
  }
  WheelSpeeds retval;
  retval.left = leftSpeed;
  retval.right = rightSpeed;
  return retval;
}

DifferentialDrive::WheelSpeeds DifferentialDrive::TankDriveIK(
    double leftSpeed, double rightSpeed, bool squareInputs) {
  leftSpeed = VpiUtils::clip(leftSpeed, -1.0, 1.0);
  rightSpeed = VpiUtils::clip(rightSpeed, -1.0, 1.0);

  // Square the inputs (while preserving the sign) to increase fine control
  // while permitting full power.
  if (squareInputs) {
    leftSpeed = std::copysign(leftSpeed * leftSpeed, leftSpeed);
    rightSpeed = std::copysign(rightSpeed * rightSpeed, rightSpeed);
  }

  WheelSpeeds retval;
  retval.left = leftSpeed;
  retval.right = rightSpeed;
  return retval;
}

void DifferentialDrive::Stop() {
  m_leftMotor->stop(m_brakeType);
  m_rightMotor->stop(m_brakeType);
}

void DifferentialDrive::DirectWheelSpeedDrive(DifferentialDriveWheelSpeeds ddws) {
  SetMotorSpeed(ddws.left, ddws.right);
}

void DifferentialDrive::TurnAngle(QAngle target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion) {
  QLength lrDistanceDiff = m_kinematics.WheelDistanceDifferenceForTurn(target);
  QSpeed s = UnitUtils::convertRotationalSpeedToLinearSpeed(turnSpeed, m_wheelDiameter, m_gearRatio);

  TurnMotorsForDistanceAtSpeed(lrDistanceDiff, -1.0 * lrDistanceDiff, s, s, waitForCompletion);
}

void DifferentialDrive::TurnToPoint(Pose2d currentPose, Point2d target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion) {
  QAngle angleToTurn = currentPose.AngleTo(target);
  TurnAngle(angleToTurn, turnSpeed, waitForCompletion);
}

void DifferentialDrive::DriveDistance(QLength target, 
                          QSpeed movementSpeed,
                          bool waitForCompletion) {
  TurnMotorsForDistanceAtSpeed(target, target, movementSpeed, movementSpeed, waitForCompletion);
}

void DifferentialDrive::DriveToPoint(Pose2d currentPose, Point2d target, 
                        QSpeed movementSpeed,
                        bool waitForCompletion) {
  QAngularSpeed s = UnitUtils::convertLinearSpeedToRotationalSpeed(movementSpeed, m_wheelDiameter, m_gearRatio);
  TurnToPoint(currentPose, target, s, true);
  QLength d = currentPose.DistanceTo(target);
  DriveDistance(d, movementSpeed, waitForCompletion);
}

bool DifferentialDrive::IsMoving()
{
  return m_leftMotor->isSpinning() || m_rightMotor->isSpinning();
}
