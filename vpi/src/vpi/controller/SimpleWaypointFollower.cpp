// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/controller/SimpleWaypointFollower.h"

namespace vpi {

void SimpleWaypointFollower::FollowTrajectoryImpl(std::vector<VexGpsPose2d> iwaypoints, 
                                                QSpeed s) {
  m_isMoving = true;
  for(auto wp : iwaypoints) {
    Point2d target(wp.X(), wp.Y());
    m_chassis.DriveToPoint(target, s, true);
  }
  m_isMoving = false;
}

void SimpleWaypointFollower::FollowTrajectory(std::vector<VexGpsPose2d> iwaypoints, 
                                                QSpeed s, bool waitForCompletion) {
  if(waitForCompletion) {
    FollowTrajectoryImpl(iwaypoints, s);
  } else {
    // Task management
    if(m_followerTask != NULL) {
      m_followerTask->stop();
    }
    this->SetWayPoints(iwaypoints);
    this->SetSpeed(s);
    m_followerTask = new vex::task(SimpleWaypointFollower::_trampoline, static_cast<void *>(this));
  }
}

int SimpleWaypointFollower::_trampoline(void *p_this) {
  SimpleWaypointFollower *p = (SimpleWaypointFollower *)p_this;
  p->FollowTrajectoryImpl();

  return 0;
}

void SimpleWaypointFollower::FollowTrajectory(std::initializer_list<VexGpsPose2d> iwaypoints, 
                                                QSpeed s, bool waitForCompletion) {
  std::vector<VexGpsPose2d> vwaypoints;
  for(auto wp : iwaypoints) {
    vwaypoints.push_back(wp);
  }
  FollowTrajectory(vwaypoints, s, waitForCompletion);
}

} // namespace vpi