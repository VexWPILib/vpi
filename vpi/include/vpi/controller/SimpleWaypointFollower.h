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
        m_chassis(chassis) {}

    void FollowTrajectory(std::vector<VexGpsPose2d> iwaypoints, QSpeed s);
    void FollowTrajectory(std::initializer_list<VexGpsPose2d> iwaypoints, QSpeed s);
  protected:
    DifferentialDriveChassis &m_chassis;
};
} // vpi
