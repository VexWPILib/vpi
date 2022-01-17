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
    if(m_debug) {
      VexGpsPose2d curStateVex = m_chassis.GetPose();
      VexGpsPose2d desiredState = states[i].pose;

      logger.log(Logger::LogLevel::DEBUG,
            "SimpleTrajectoryFollower::FollowTrajectory - state index %d - "
            "\n\tCurrent Pose : (%d, %d) H: %d"
            "\n\tTarget Pose  : (%d, %d) H: %d"
            "\n\tTarget:   vx %.3lf omega %.3lf acc %.3lf"
            "\n\tAdjusted: vx %.3lf omega %.3lf",
            i, 
            (int)curStateVex.X().convert(inch), (int)curStateVex.Y().convert(inch),
            (int)curStateVex.Theta().convert(degree),
            (int)desiredState.X().convert(inch), (int)desiredState.Y().convert(inch),
            (int)desiredState.Theta().convert(degree),
            states[i].velocity.convert(ftps), 
            states[i].velocity.convert(mps) * states[i].curvature.convert(radpermeter),
            states[i].acceleration.convert(ftps2),
            cs.vx.convert(ftps), cs.omega.convert(radps)
            );
    }
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