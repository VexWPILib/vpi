// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/controller/RamseteController.h"
#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/trajectory/Trajectory.h"
#include "vpi/units/QTime.h"

namespace vpi {
  /**
   * For each point, simply turns to face the next point, then drives to it
   * stopping at each point.
   */
class RamseteTrajectoryFollower {
  public:
    RamseteTrajectoryFollower(DifferentialDriveChassis &chassis,
                              Pose2d tolerance,
                              double b=2.0, double zeta=0.7) :
        m_chassis(chassis), m_rc(b, zeta) {
      m_rc.SetTolerance(tolerance);
    }

    void FollowTrajectory(Trajectory t);

  protected:
    DifferentialDriveChassis &m_chassis;
    RamseteController m_rc;
};
} // vpi
