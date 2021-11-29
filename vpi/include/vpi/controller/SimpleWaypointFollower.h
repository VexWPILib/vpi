// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/geometry/Point2d.h"
#include "vpi/geometry/Pose2d.h"
#include "vpi/geometry/VexGpsPose2d.h"

namespace vpi {
  /**
   * For each point, simply turns to face the next point, then drives to it
   * stopping at each point.
   */
class SimpleWaypointFollower {
  public:
    SimpleWaypointFollower(DifferentialDriveChassis &chassis) :
        m_chassis(chassis), m_isMoving(false) {}

    void FollowTrajectory(std::vector<VexGpsPose2d> iwaypoints, QSpeed s, bool waitForCompletion=true);
    void FollowTrajectory(std::initializer_list<VexGpsPose2d> iwaypoints, QSpeed s, bool waitForCompletion=true);
    virtual bool IsMoving() { return m_isMoving;}

  protected:
    DifferentialDriveChassis &m_chassis;
    bool m_isMoving;

    virtual void FollowTrajectoryImpl(std::vector<VexGpsPose2d> iwaypoints, QSpeed s);
    void FollowTrajectoryImpl() { FollowTrajectoryImpl(m_waypoints, m_speed);}

    void SetWayPoints(std::vector<VexGpsPose2d> &iwaypoints) {
        m_waypoints = iwaypoints;
    }

    void SetSpeed(QSpeed s) {
        m_speed = s;
    }

  private:
    vex::task *m_followerTask = NULL;
    static int _trampoline(void *p_this);
    std::vector<VexGpsPose2d> m_waypoints;
    QSpeed m_speed;
};
} // vpi
