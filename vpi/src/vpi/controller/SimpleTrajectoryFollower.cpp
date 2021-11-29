// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/controller/SimpleTrajectoryFollower.h"

namespace vpi {

void SimpleTrajectoryFollower::FollowTrajectoryImpl(Trajectory t) {
  m_isMoving = true;
  std::vector<Trajectory::State> states = t.States();
  for(int i = 1; i < states.size(); ++i) {
    ChassisSpeeds cs;
    cs.vx = states[i].velocity;
    cs.omega = states[i].velocity * states[i].curvature;
    m_chassis.DriveChassisSpeeds(cs);
    QTime timeToWait = states[i].t - states[i - 1].t;
    wait(timeToWait.convert(millisecond), msec);
  }
  m_isMoving = false;
}

void SimpleTrajectoryFollower::FollowTrajectory(Trajectory t, bool waitForCompletion) {
  if(waitForCompletion) {
    FollowTrajectoryImpl(t);
  } else {
    // Task management
    if(m_followerTask != NULL) {
      m_followerTask->stop();
    }
    this->SetTrajectory(t);
    m_followerTask = new vex::task(SimpleTrajectoryFollower::_trampoline, static_cast<void *>(this));
  }
}

int SimpleTrajectoryFollower::_trampoline(void *p_this) {
  SimpleTrajectoryFollower *p = (SimpleTrajectoryFollower *)p_this;
  p->FollowTrajectoryImpl();

  return 0;
}

} // namespace vpi