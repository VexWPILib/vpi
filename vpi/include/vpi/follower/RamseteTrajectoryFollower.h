// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/controller/RamseteController.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/log/Logger.h"
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
        m_chassis(chassis), m_rc(b, zeta), m_debug(false) {
      m_rc.SetTolerance(tolerance);
      if(!m_chassis.IsOdomEnabled()) {
        m_chassis.EnableOdom();
      }
    }

    void FollowTrajectory(Trajectory t, bool waitForCompletion=true);

    bool IsMoving() {return m_isMoving;}
    virtual void SetDebug(bool b) {m_debug = b;}

  protected:
    DifferentialDriveChassis &m_chassis;
    RamseteController m_rc;
    bool m_isMoving;
    bool m_debug;

    virtual void FollowTrajectoryImpl(Trajectory t);
    void FollowTrajectoryImpl() { FollowTrajectoryImpl(m_trajectory);}

    void SetTrajectory(Trajectory &t) {
        m_trajectory = t;
    }

  private:
    vex::task *m_followerTask = NULL;
    static int _trampoline(void *p_this);
    Trajectory m_trajectory;

};
} // vpi
