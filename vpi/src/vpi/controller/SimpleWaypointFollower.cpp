// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/controller/SimpleWaypointFollower.h"

namespace vpi {

void SimpleWaypointFollower::FollowTrajectory(std::vector<Pose2d> iwaypoints, 
                                                QSpeed s) {
  for(auto wp : iwaypoints) {
    Point2d target(wp.X(), wp.Y());
    m_chassis.DriveToPoint(target, s, true);
  }
}

void SimpleWaypointFollower::FollowTrajectory(std::initializer_list<Pose2d> iwaypoints, 
                                                QSpeed s) {
  for(auto wp : iwaypoints) {
    Point2d target(wp.X(), wp.Y());
    m_chassis.DriveToPoint(target, s, true);
  }
}

} // namespace vpi