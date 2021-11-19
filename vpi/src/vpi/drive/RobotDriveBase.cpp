// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/drive/RobotDriveBase.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {
  QAngle RobotDriveBase::computeMotorRotationsForTurn(Pose2d currentPose, Point2d target) {
    Point2d p = Point2d(currentPose.X(), currentPose.Y());
    // return pthis.computeDistanceToPoint(pend);
    QAngle turnAmount = UnitUtils::constrainTo180(p.computeAngleToPoint(target));
    turnAmount = UnitUtils::constrainTo180(turnAmount - currentPose.Rotation().ToAngle());
    QLength wheelLinearDistance = turnAmount.convert(radian) * m_wheelTrack / 2.0;
    return computeMotorRotationsForDrive(wheelLinearDistance);
  }

  QAngle RobotDriveBase::computeMotorRotationsForDrive(QLength target) {
    return UnitUtils::convertLinearToRotational(target, m_wheelDiameter, m_gearRatio);
  }
} // end vpi