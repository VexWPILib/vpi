// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/log/Logger.h"
#include "vpi/trajectory/Trajectory.h"
#include "vpi/units/QTime.h"

namespace vpi {
  /**
   * Basic pure-pursuit implementation using a user-configured
   * look-ahead distance.
   */
class DifferentialDrivePurePursuitFollower {
  public:
    DifferentialDrivePurePursuitFollower(DifferentialDriveChassis &chassis, 
                                          QLength maxLookAhead,
                                          QSpeed maxSpeed,
                                          QAcceleration maxAcc) :
        m_chassis(chassis), m_maxLookAhead(maxLookAhead),
        m_maxSpeed(maxSpeed), m_maxAcc(maxAcc) {

      if(!m_chassis.IsOdomEnabled()) {
        m_chassis.EnableOdom();
      }
    }

    void FollowTrajectory(Trajectory t, bool waitForCompletion=true);

    bool IsMoving() {return m_isMoving;}
    virtual void SetDebug(bool b) {m_debug = b;}
    void SetStepDuration(QTime t) {m_stepDuration = t;}

  protected:
    DifferentialDriveChassis &m_chassis;
    QLength m_maxLookAhead;
    QSpeed m_maxSpeed;
    QAcceleration m_maxAcc;
    bool m_isMoving;
    bool m_debug;
    int m_curPathIndex = 1;
    QTime m_stepDuration = 20_ms;

    VexGpsPose2d FindLookaheadPoint(Trajectory t);

    virtual void FollowTrajectoryImpl(Trajectory t);
    void FollowTrajectoryImpl() { FollowTrajectoryImpl(m_trajectory);}

    void SetTrajectory(Trajectory &t) {
        m_trajectory = t;
        m_curPathIndex = 1;
    }

  private:
    vex::task *m_followerTask = NULL;
    static int _trampoline(void *p_this);
    Trajectory m_trajectory;

    int FindClosestPathPointIndex(Trajectory t, int startIndex);
    int IsDone();
};
} // vpi
