// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/drive/MecanumDrive.h"
#include "vpi/log/Logger.h"
#include "vpi/utils.h"
#include "vpi/geometry/Vector2d.h"

using namespace vpi;

void MecanumDrive::SetMotorSpeedPercent(double leftFrontSpeed, double rightFrontSpeed,
                                        double leftRearSpeed, double rightRearSpeed)
{
  // TODO - any filtering of the speeds
  if(fabs(leftFrontSpeed) > 1.0 || fabs(leftRearSpeed) > 1.0 ||
      fabs(rightFrontSpeed) > 1.0 || fabs(rightRearSpeed) > 1.0 ) {
    logger.log(Logger::LogLevel::WARN, "SetMotorSpeedPercent has value over 1.0 - LF: %.3lf LR: %.3lf RF: %.3lf RR: %.3lf",
              leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed);
  }
  if(leftFrontSpeed > 0) {
    m_frontLeftMotor->spin(vex::directionType::fwd, fabs(leftFrontSpeed) * 12.0, vex::voltageUnits::volt);
  } else {
    m_frontLeftMotor->spin(vex::directionType::rev, fabs(leftFrontSpeed) * 12.0, vex::voltageUnits::volt);
  }
  if(leftRearSpeed > 0) {
    m_rearLeftMotor->spin(vex::directionType::fwd, fabs(leftRearSpeed) * 12.0, vex::voltageUnits::volt);
  } else {
    m_rearLeftMotor->spin(vex::directionType::rev, fabs(leftRearSpeed) * 12.0, vex::voltageUnits::volt);
  }
  if(rightFrontSpeed > 0) {
    m_frontRightMotor->spin(vex::directionType::fwd, fabs(rightFrontSpeed) * 12.0, vex::voltageUnits::volt);
  } else {
    m_frontRightMotor->spin(vex::directionType::rev, fabs(rightFrontSpeed) * 12.0, vex::voltageUnits::volt);
  }
  if(rightRearSpeed > 0) {
    m_rearRightMotor->spin(vex::directionType::fwd, fabs(rightRearSpeed) * 12.0, vex::voltageUnits::volt);
  } else {
    m_rearRightMotor->spin(vex::directionType::rev, fabs(rightRearSpeed) * 12.0, vex::voltageUnits::volt);
  }
}

void MecanumDrive::SetMotorSpeed(QSpeed leftFrontSpeed, QSpeed rightFrontSpeed,
                                QSpeed leftRearSpeed, QSpeed rightRearSpeed)
{
  // TODO - any filtering of the speeds
  double leftFrontRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(leftFrontSpeed, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rightFrontRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(rightFrontSpeed, m_wheelDiameter, m_gearRatio).convert(rpm);
  double leftRearRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(leftRearSpeed, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rightRearRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(rightRearSpeed, m_wheelDiameter, m_gearRatio).convert(rpm);

  m_frontLeftMotor->spin(vex::directionType::fwd, leftFrontRpm, vex::velocityUnits::rpm);
  m_frontRightMotor->spin(vex::directionType::fwd, rightFrontRpm, vex::velocityUnits::rpm);
  m_rearLeftMotor->spin(vex::directionType::fwd, leftRearRpm, vex::velocityUnits::rpm);
  m_rearRightMotor->spin(vex::directionType::fwd, rightRearRpm, vex::velocityUnits::rpm);
}

void MecanumDrive::TurnMotorsForDistanceAtSpeed(QLength lfd, QLength rfd, 
                                QLength lrd, QLength rrd,
                                QSpeed lfs, QSpeed rfs,
                                QSpeed lrs, QSpeed rrs,
                                bool waitForCompletion) {
  double leftFrontDegrees = UnitUtils::convertLinearToRotational(lfd, m_wheelDiameter, m_gearRatio).convert(degree);
  double rightFrontDegrees = UnitUtils::convertLinearToRotational(rfd, m_wheelDiameter, m_gearRatio).convert(degree);
  double leftRearDegrees = UnitUtils::convertLinearToRotational(lrd, m_wheelDiameter, m_gearRatio).convert(degree);
  double rightRearDegrees = UnitUtils::convertLinearToRotational(rrd, m_wheelDiameter, m_gearRatio).convert(degree);

  // TODO - any filtering of the speeds
  double leftFrontRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(lfs, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rightFrontRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(rfs, m_wheelDiameter, m_gearRatio).convert(rpm);
  double leftRearRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(lrs, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rightRearRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(rrs, m_wheelDiameter, m_gearRatio).convert(rpm);

  m_frontLeftMotor->spinFor(leftFrontDegrees, vex::rotationUnits::deg, leftFrontRpm, vex::velocityUnits::rpm, false);
  m_frontRightMotor->spinFor(rightFrontDegrees, vex::rotationUnits::deg, rightFrontRpm, vex::velocityUnits::rpm, false);
  m_rearLeftMotor->spinFor(leftRearDegrees, vex::rotationUnits::deg, leftRearRpm, vex::velocityUnits::rpm, false);
  m_rearRightMotor->spinFor(rightRearDegrees, vex::rotationUnits::deg, rightRearRpm, vex::velocityUnits::rpm, false);

  if(waitForCompletion) {
    while(!m_frontLeftMotor->isDone() && !m_frontRightMotor->isDone() &&
          !m_rearLeftMotor->isDone() && !m_rearRightMotor->isDone()) {
      wait(20, msec);
    }
    Stop();
  }
}

void MecanumDrive::DriveCartesian(double ySpeed, double xSpeed,
                                  double zRotation, double gyroAngle) {
  ySpeed = VpiUtils::ApplyDeadband(ySpeed, m_deadband);
  xSpeed = VpiUtils::ApplyDeadband(xSpeed, m_deadband);

  WheelSpeeds ws = DriveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);

  m_frontLeftMotor->spin(vex::directionType::fwd, ws.frontLeft * 100.0, vex::percentUnits::pct);
  m_frontRightMotor->spin(vex::directionType::fwd, ws.frontRight * 100.0, vex::percentUnits::pct);
  m_rearLeftMotor->spin(vex::directionType::fwd, ws.rearLeft * 100.0, vex::percentUnits::pct);
  m_rearRightMotor->spin(vex::directionType::fwd, ws.rearRight * 100.0, vex::percentUnits::pct);

}

void MecanumDrive::DrivePolar(double magnitude, double angle,
                              double zRotation) {
  DriveCartesian(magnitude * std::cos(angle * (M_PI / 180.0)),
                 magnitude * std::sin(angle * (M_PI / 180.0)),
                 zRotation, 0.0);
}

void MecanumDrive::Stop() {
  m_frontLeftMotor->stop(m_brakeType);
  m_frontRightMotor->stop(m_brakeType);
  m_rearLeftMotor->stop(m_brakeType);
  m_rearRightMotor->stop(m_brakeType);
}

MecanumDrive::WheelSpeeds MecanumDrive::DriveCartesianIK(double ySpeed,
                                                         double xSpeed,
                                                         double zRotation,
                                                         double gyroAngle) {
  ySpeed = VpiUtils::clip(ySpeed, -1.0, 1.0);
  xSpeed = VpiUtils::clip(xSpeed, -1.0, 1.0);

  // Compensate for gyro angle.
  Vector2d input{ySpeed, xSpeed};
  input.Rotate(-gyroAngle);

  WheelSpeeds ws;
  ws.frontLeft = input.x + input.y + zRotation;
  ws.frontRight = input.x - input.y - zRotation;
  ws.rearLeft = input.x - input.y + zRotation;
  ws.rearRight = input.x + input.y - zRotation;

  return ws;
}

void MecanumDrive::DirectWheelSpeedDrive(MecanumDriveWheelSpeeds ddws) {
  double frontLeftRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(ddws.frontLeft, m_wheelDiameter, m_gearRatio).convert(rpm);
  double frontRightRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(ddws.frontRight, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rearLeftRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(ddws.rearLeft, m_wheelDiameter, m_gearRatio).convert(rpm);
  double rearRightRpm = UnitUtils::convertLinearSpeedToRotationalSpeed(ddws.rearRight, m_wheelDiameter, m_gearRatio).convert(rpm);

  m_frontLeftMotor->spin(vex::directionType::fwd, frontLeftRpm, vex::velocityUnits::rpm);
  m_frontRightMotor->spin(vex::directionType::fwd, frontRightRpm, vex::velocityUnits::rpm);
  m_rearLeftMotor->spin(vex::directionType::fwd, rearLeftRpm, vex::velocityUnits::rpm);
  m_rearRightMotor->spin(vex::directionType::fwd, rearRightRpm, vex::velocityUnits::rpm);
}

void MecanumDrive::TurnAngle(QAngle target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion) {
  QLength lrDistanceDiff = m_kinematics.WheelDistanceDifferenceForTurn(target);
  QSpeed s = UnitUtils::convertRotationalSpeedToLinearSpeed(turnSpeed, m_wheelDiameter, m_gearRatio);

  TurnMotorsForDistanceAtSpeed(lrDistanceDiff, -1.0 * lrDistanceDiff, lrDistanceDiff, -1.0 * lrDistanceDiff,
                              s, s, s, s, waitForCompletion);
}

void MecanumDrive::TurnToPoint(VexGpsPose2d currentPose, VexGpsPose2d target, 
                          QAngularSpeed turnSpeed,
                          bool waitForCompletion) {
  QAngle angleToTurn = ((Pose2d)currentPose).AngleTo({target.X(), target.Y()});
  angleToTurn = UnitUtils::constrainTo180(angleToTurn - currentPose.Theta());
  TurnAngle(angleToTurn, turnSpeed, waitForCompletion);
}

void MecanumDrive::DriveDistance(QLength target, 
                          QSpeed movementSpeed,
                          bool waitForCompletion) {
 TurnMotorsForDistanceAtSpeed(target, target, target, target, 
                            movementSpeed, movementSpeed, movementSpeed, movementSpeed,
                            waitForCompletion);
}

void MecanumDrive::DriveToPoint(VexGpsPose2d currentPose, VexGpsPose2d target, 
                        QSpeed movementSpeed,
                        bool waitForCompletion) {
  QAngularSpeed s = UnitUtils::convertLinearSpeedToRotationalSpeed(movementSpeed, m_wheelDiameter, m_gearRatio);
  TurnToPoint(currentPose, target, s, true);
  QLength d = ((Pose2d)currentPose).DistanceTo({target.X(), target.Y()});
  DriveDistance(d, movementSpeed, waitForCompletion);
}

bool MecanumDrive::IsMoving()
{
  return m_frontLeftMotor->isSpinning() || m_frontRightMotor->isSpinning() ||
        m_rearLeftMotor->isSpinning() || m_rearRightMotor->isSpinning();
}