// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

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
class SimpleTrajectoryFollower {
  public:
    SimpleTrajectoryFollower(DifferentialDriveChassis &chassis) :
        m_chassis(chassis) {}

    void FollowTrajectory(Trajectory t, bool waitForCompletion = true);

    bool IsMoving() {return m_isMoving;}
    
  protected:
    DifferentialDriveChassis &m_chassis;
    bool m_isMoving;

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
