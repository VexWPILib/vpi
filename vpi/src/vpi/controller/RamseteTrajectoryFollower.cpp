// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/controller/RamseteTrajectoryFollower.h"

using namespace vpi;

namespace vpi {

void RamseteTrajectoryFollower::FollowTrajectory(Trajectory t) {
  std::vector<Trajectory::State> states = t.States();
  for(int i = 1; i < states.size(); ++i) {
    ChassisSpeeds cs = m_rc.Calculate(m_chassis.GetPose(), states[i]);
    m_chassis.DriveChassisSpeeds(cs);
    QTime timeToWait = states[i].t - states[i - 1].t;
    wait(timeToWait.convert(millisecond), msec);
  } 
}

} // namespace vpi