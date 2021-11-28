// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/controller/RamseteTrajectoryFollower.h"

using namespace vpi;

namespace vpi {

void RamseteTrajectoryFollower::FollowTrajectoryImpl(Trajectory t) {
  m_isMoving = true;
  std::vector<Trajectory::State> states = t.States();
  for(int i = 1; i < states.size(); ++i) {
    Pose2d curState = m_chassis.GetPose();
    VexGpsPose2d curStateVex = curState;
    VexGpsPose2d desiredStateVex = states[i].pose;
    ChassisSpeeds cs = m_rc.Calculate(m_chassis.GetPose(), states[i]);
    logger.log(Logger::LogLevel::DEBUG,
          "RamseteTrajectoryFollower::FollowTrajectory - state index %d - "
          "\n\tCurrent Pose : (%d, %d) H: %d"
          "\n\tTarget: (%d, %d) H %d vx %.3lf omega %.3lf acc %.3lf"
          "\n\tAdjusted: vx %.3lf omega %.3lf",
          i, 
          (int)curStateVex.X().convert(inch), (int)curStateVex.Y().convert(inch),
          (int)curStateVex.Theta().convert(degree),
          (int)desiredStateVex.X().convert(inch), (int)desiredStateVex.Y().convert(inch),
          (int)desiredStateVex.Theta().convert(degree),
          states[i].velocity.convert(ftps), 
          states[i].velocity.convert(mps) * states[i].curvature.convert(radpermeter),
          states[i].acceleration.convert(ftps2),
          cs.vx.convert(ftps), cs.omega.convert(radps)
          );

    m_chassis.DriveChassisSpeeds(cs);
    QTime timeToWait = states[i].t - states[i - 1].t;
    wait(timeToWait.convert(millisecond), msec);
  } 
  m_isMoving = false;
}

void RamseteTrajectoryFollower::FollowTrajectory(Trajectory t, bool waitForCompletion) {
  if(waitForCompletion) {
    FollowTrajectoryImpl(t);
  } else {
    // Task management
    if(m_followerTask != NULL) {
      m_followerTask->stop();
    }
    this->SetTrajectory(t);
    m_followerTask = new vex::task(RamseteTrajectoryFollower::_trampoline, static_cast<void *>(this));
  }
}

int RamseteTrajectoryFollower::_trampoline(void *p_this) {
  RamseteTrajectoryFollower *p = (RamseteTrajectoryFollower *)p_this;
  p->FollowTrajectoryImpl();

  return 0;
}

} // namespace vpi