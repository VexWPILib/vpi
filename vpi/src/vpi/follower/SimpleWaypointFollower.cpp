// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/follower/SimpleWaypointFollower.h"

namespace vpi {

void SimpleWaypointFollower::FollowTrajectoryImpl(std::vector<VexGpsPose2d> iwaypoints, 
                                                QSpeed s) {
  m_isMoving = true;
  for(auto wp : iwaypoints) {
    m_chassis.DriveToPoint(wp, s, true);
    if(m_debug) {
      VexGpsPose2d curStateVex = m_chassis.GetPose();

      logger.log(Logger::LogLevel::DEBUG, 
            "SimpleWaypointFollower::FollowTrajectory - "
            "\n\tCurrent Pose : (%d, %d) H: %d"
            "\n\tTarget Pose  : (%d, %d)",
                (int)curStateVex.X().convert(inch),
                (int)curStateVex.Y().convert(inch),
                (int)curStateVex.Theta().convert(degree),
                (int)wp.X().convert(inch), (int)wp.Y().convert(inch));
    }
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