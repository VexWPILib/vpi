// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vex.h"
#include "vpi/chassis/DifferentialDriveChassisPID.h"

namespace vpi {
DifferentialDriveChassisPID::DifferentialDriveChassisPID(vex::motor_group& leftMotorGroup, 
                          vex::motor_group& rightMotorGroup,
                          QLength driveTrackWidth,
                          QLength driveWheelDiameter,
                          PIDFParameters pDistance, PIDFParameters pAngle, 
                          PIDFParameters pTurn,
                          vex::gearSetting gs,
                          double gearRatio, vex::brakeType brake) :
      DifferentialDriveChassis(leftMotorGroup, rightMotorGroup, driveTrackWidth, driveWheelDiameter,
                              gs, gearRatio, brake)
{
  m_pidDistanceController = new PIDController(pDistance, [this] {return this->DistanceToTarget().convert(inch);},
                                              [this](double v) {this->ConsumeDistancePID(v);});
  m_pidDistanceController->SetTolerance(0.5);  // Half an inch tolerance since DistanceToTarget().convert(inch)
  m_pidAngleController = new PIDController(pAngle, [this] {return UnitUtils::constrainTo180(this->AngleToTarget()).convert(degree);},
                                              [this](double v) {this->ConsumeAnglePID(v);});
  m_pidAngleController->EnableContinuousInput(-180, 180);
  m_pidTurnController = new PIDController(pTurn, 
                                            [this] {return UnitUtils::constrainTo180(this->AngleToTarget()).convert(degree);},
                                            [this](double v) {this->ConsumeTurnPID(v);});
  m_pidTurnController->EnableContinuousInput(-180, 180);
                                        
}

void DifferentialDriveChassisPID::Reset()
{
  m_mutex.lock();
  m_pidDistanceController->SetEnabled(false);
  m_pidAngleController->SetEnabled(false);
  m_pidTurnController->SetEnabled(false);
  m_pidDistanceController->Reset();
  m_pidAngleController->Reset();
  m_pidTurnController->Reset();
  m_startPoint = NULL;
  m_targetPoint = NULL;
  m_targetSpeed = NULL;
  m_targetAngularSpeed = NULL;
  m_desiredWheelSpeeds = NULL;
  m_mutex.unlock();
  m_drivetrain.Stop();
}

QLength DifferentialDriveChassisPID::DistanceToTarget() {
  m_mutex.lock();
  if(m_targetPoint != NULL) {
    // We can't just do GetPose().DistanceTo(*m_targetPoint)
    // as when we overshoot, this will still give a positive
    // distance
    Pose2d p = GetPose();
    QLength distToTarget = p.DistanceTo(*m_targetPoint);
    QLength startDistance = m_startPoint->computeDistanceToPoint(*m_targetPoint);
    QLength distanceTravelled = p.DistanceTo(*m_startPoint);
    if(distanceTravelled > startDistance) {
      distToTarget = -1.0 * distToTarget;
    }
    return distToTarget;
  } else {
    return 0_m;
  }
  m_mutex.unlock();
}

QAngle DifferentialDriveChassisPID::AngleToTarget() {
  m_mutex.lock();
  if(m_targetPoint != NULL) {
    // If we're less than 6 inches to the target, don't adjust
    if(fabs(GetPose().DistanceTo(*m_targetPoint).convert(inch)) < 4) {
      return 0_rad;
    } else {
      return GetPose().AngleTo(*m_targetPoint);
    }
  } else {
    return 0_rad;
  }
  m_mutex.unlock();
}

void DifferentialDriveChassisPID::TurnToPoint(Point2d target, 
                        QAngularSpeed turnSpeed, 
                        bool waitForCompletion)
{
  m_mutex.lock();
  m_pidDistanceController->SetEnabled(false);
  m_pidAngleController->SetEnabled(false);
  m_pidTurnController->SetEnabled(false);
  m_targetPoint = new Point2d(target);
  Pose2d p = GetPose();
  m_startPoint = new Point2d(p.X(), p.Y());
  m_targetSpeed = NULL;
  m_targetAngularSpeed = new QAngularSpeed(turnSpeed);
  m_desiredWheelSpeeds = new DifferentialDriveWheelSpeeds();
  QSpeed s = UnitUtils::convertRotationalSpeedToLinearSpeed(*m_targetAngularSpeed, m_driveWheelDiameter, m_gearRatio);
  m_desiredWheelSpeeds->left = s;
  m_desiredWheelSpeeds->right = -1.0 * s;
  m_mutex.unlock();
  m_pidTurnController->SetEnabled(true);
  if(waitForCompletion) {
    while(m_pidTurnController->IsEnabled() && !m_pidTurnController->AtSetpoint()) {
      this_thread::sleep_for(m_pidTurnController->GetPeriod().convert(millisecond));
    }
    Reset();
  }
}

void DifferentialDriveChassisPID::ConsumeTurnPID(double a)
{
  m_mutex.lock();
  QSpeed s = UnitUtils::convertRotationalSpeedToLinearSpeed(*m_targetAngularSpeed * a, m_driveWheelDiameter, m_gearRatio);
  // TODO - A SlewRateFilter could be applied here
  m_desiredWheelSpeeds->left = s;
  m_desiredWheelSpeeds->right = -1.0 * s;
  m_mutex.unlock();

  m_drivetrain.DirectWheelSpeedDrive(*m_desiredWheelSpeeds);
}

void DifferentialDriveChassisPID::DriveDistance(QLength target, 
                        QSpeed movementSpeed, 
                        bool waitForCompletion)
{
  m_mutex.lock();
  m_pidDistanceController->SetEnabled(false);
  m_pidAngleController->SetEnabled(false);
  m_pidTurnController->SetEnabled(false);
  Translation2d t(target, GetPose().Rotation().ToAngle());
  Transform2d f(t, 0_rad);
  Pose2d p = GetPose();
  m_startPoint = new Point2d(p.X(), p.Y());
  Pose2d targetPose = p + f;
  m_targetPoint = new Point2d(targetPose.X(), targetPose.Y());
  m_targetSpeed = new QSpeed(movementSpeed);
  m_targetAngularSpeed = NULL;
  m_desiredWheelSpeeds = new DifferentialDriveWheelSpeeds();
  m_desiredWheelSpeeds->left = movementSpeed;
  m_desiredWheelSpeeds->right = movementSpeed;
  m_mutex.unlock();
  m_pidDistanceController->SetEnabled(true);
  m_pidAngleController->SetEnabled(true);
  if(waitForCompletion) {
    while(m_pidDistanceController->IsEnabled() && !m_pidDistanceController->AtSetpoint()) {
      this_thread::sleep_for(m_pidDistanceController->GetPeriod().convert(millisecond));
    }
    Reset();
  }
}

void DifferentialDriveChassisPID::ConsumeDistancePID(double v)
{
  m_mutex.lock();
  QSpeed s = *m_targetSpeed * v;
  // TODO - A SlewRateFilter could be applied here
  m_desiredWheelSpeeds->left = s;
  m_desiredWheelSpeeds->right = s;
  m_desiredWheelSpeeds->Normalize(*m_targetSpeed);
  m_mutex.unlock();

  m_drivetrain.DirectWheelSpeedDrive(*m_desiredWheelSpeeds);
}

void DifferentialDriveChassisPID::ConsumeAnglePID(double a)
{
  m_mutex.lock();
  m_desiredWheelSpeeds->left = m_desiredWheelSpeeds->left * a;
  m_desiredWheelSpeeds->right = m_desiredWheelSpeeds->right * a;
  m_desiredWheelSpeeds->Normalize(*m_targetSpeed);
  m_mutex.unlock();

  m_drivetrain.DirectWheelSpeedDrive(*m_desiredWheelSpeeds);
}

void DifferentialDriveChassisPID::DriveToPoint(Point2d target, 
                        QSpeed movementSpeed, 
                        bool waitForCompletion)
{
  QAngularSpeed turnSpeed = UnitUtils::convertLinearSpeedToRotationalSpeed(movementSpeed,
                                                                          m_driveWheelDiameter,
                                                                          m_gearRatio);
  TurnToPoint(target, turnSpeed, true);

  m_mutex.lock();
  m_pidDistanceController->SetEnabled(false);
  m_pidAngleController->SetEnabled(false);
  m_pidTurnController->SetEnabled(false);
  Pose2d p = GetPose();
  m_startPoint = new Point2d(p.X(), p.Y());
  m_targetPoint = new Point2d(target);
  m_targetSpeed = new QSpeed(movementSpeed);
  m_targetAngularSpeed = NULL;
  m_desiredWheelSpeeds = new DifferentialDriveWheelSpeeds();
  m_desiredWheelSpeeds->left = movementSpeed;
  m_desiredWheelSpeeds->right = movementSpeed;
  m_mutex.unlock();
  m_pidDistanceController->SetEnabled(true);
  m_pidAngleController->SetEnabled(true);
  if(waitForCompletion) {
    while(m_pidDistanceController->IsEnabled() && !m_pidDistanceController->AtSetpoint()) {
      this_thread::sleep_for(m_pidDistanceController->GetPeriod().convert(millisecond));
    }
    Reset();
  }
}
} // namespace vpi