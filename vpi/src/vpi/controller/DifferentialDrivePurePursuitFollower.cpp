// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/controller/DifferentialDrivePurePursuitFollower.h"

namespace vpi {

void DifferentialDrivePurePursuitFollower::FollowTrajectoryImpl(Trajectory t) {
  m_isMoving = true;
  std::vector<Trajectory::State> states = t.States();
  do {
    VexGpsPose2d curState = m_chassis.GetPose();
    int iNearestIndex = FindClosestPathPointIndex(t,m_curPathIndex);
    QLength dNearest = curState.DistanceTo(states[iNearestIndex].pose);
    bool bOnPath = (dNearest < m_maxLookAhead);
    VexGpsPose2d lookAheadPoint = FindLookaheadPoint(t);
    QCurvature c = curState.CurvatureToPoint(lookAheadPoint, m_maxLookAhead);
    ChassisSpeeds cs;

    if(bOnPath) {
      cs.vx = states[iNearestIndex].velocity;
    } else {
      QSpeed curVel = (m_chassis.LeftMotorSpeed() + m_chassis.LeftMotorSpeed()) / 2.0;
      if(fabs(curVel.convert(ftps)) < fabs(m_maxSpeed.convert(ftps))) {
        curVel += m_maxAcc * m_stepDuration;
        if(fabs(curVel.convert(ftps)) > fabs(m_maxSpeed.convert(ftps))) {
          curVel = m_maxSpeed;
        }
      }
      cs.vx = curVel;
    }
    cs.omega = cs.vx * c;
    m_chassis.DriveChassisSpeeds(cs);

    if(m_debug) {
      VexGpsPose2d desiredState = states[iNearestIndex].pose;

      logger.log(Logger::LogLevel::DEBUG,
            "DifferentialDrivePurePursuitFollower::FollowTrajectory - state index %d - "
            "\n\tCurrent Pose : (%d, %d) H: %d"
            "\n\tTarget Pose  : (%d, %d) H: %d"
            "\n\tTarget:   vx %.3lf omega %.3lf acc %.3lf"
            "\n\tAdjusted: vx %.3lf omega %.3lf",
            iNearestIndex, 
            (int)curState.X().convert(inch), (int)curState.Y().convert(inch),
            (int)curState.Theta().convert(degree),
            (int)desiredState.X().convert(inch), (int)desiredState.Y().convert(inch),
            (int)desiredState.Theta().convert(degree),
            states[iNearestIndex].velocity.convert(ftps), 
            states[iNearestIndex].velocity.convert(mps) * states[iNearestIndex].curvature.convert(radpermeter),
            states[iNearestIndex].acceleration.convert(ftps2),
            cs.vx.convert(ftps), cs.omega.convert(radps)
            );
    }
    wait(m_stepDuration.convert(millisecond), msec);
  } while(!IsDone());
  m_isMoving = false;
}

void DifferentialDrivePurePursuitFollower::FollowTrajectory(Trajectory t, bool waitForCompletion) {
  if(waitForCompletion) {
    FollowTrajectoryImpl(t);
  } else {
    // Task management
    if(m_followerTask != NULL) {
      m_followerTask->stop();
    }
    this->SetTrajectory(t);
    m_followerTask = new vex::task(DifferentialDrivePurePursuitFollower::_trampoline, static_cast<void *>(this));
  }
}

int DifferentialDrivePurePursuitFollower::_trampoline(void *p_this) {
  DifferentialDrivePurePursuitFollower *p = (DifferentialDrivePurePursuitFollower *)p_this;
  p->FollowTrajectoryImpl();

  return 0;
}

int DifferentialDrivePurePursuitFollower::FindClosestPathPointIndex(Trajectory t, int startIndex) {
  VexGpsPose2d curState = m_chassis.GetPose();
  
  std::vector<Trajectory::State> states = t.States();

  QLength curMinDistance = 999 * meter;
  int bestIndex = startIndex;

  // Find the point on the path closest to the robot
  for(int i = startIndex; i < states.size() - 1; ++i) {
    VexGpsPose2d pathPoint = states[i].pose;
    QLength d = curState.DistanceTo(pathPoint);
    if(d < curMinDistance) {
      curMinDistance = d;
      bestIndex = i;
    }
  }
  
  return bestIndex;
}

int DifferentialDrivePurePursuitFollower::IsDone() {
  std::vector<Trajectory::State> states = m_trajectory.States();
  int nearestIndex = FindClosestPathPointIndex(m_trajectory, 0);
  return (nearestIndex == states.size() - 1);
}

VexGpsPose2d DifferentialDrivePurePursuitFollower::FindLookaheadPoint(Trajectory t) {
  VexGpsPose2d curState = m_chassis.GetPose();
  
  std::vector<Trajectory::State> states = t.States();
  VexGpsPose2d endPoint = states[states.size() - 1].pose;  // Default to the start of the path
  if(curState.DistanceTo(endPoint) < m_maxLookAhead) {
    // If the endpoint is closer than the lookahead distance just go there
    m_curPathIndex = states.size() - 1;
    return endPoint;
  }

  m_curPathIndex = FindClosestPathPointIndex(t, m_curPathIndex);

  // Loop through path from this nearest point to find the line
  // segment on the path that intersects with the lookahead distance  
  std::vector<double> tval;
  int bestPathPointIndex = m_curPathIndex;
  for(int i = m_curPathIndex; i < states.size() - 1; ++i) {
    VexGpsPose2d pathPoint = states[i].pose;
    VexGpsPose2d nextPathPoint = states[i + 1].pose;
    tval = curState.CircleLineIntersection(m_maxLookAhead, pathPoint, nextPathPoint);
    if(tval.size() > 0) {
      bestPathPointIndex = i;
      break;
    }
  }
  double tv = 1.0;

  if(tval.size() != 0) {
    tv = tval[0];
  }
  VexGpsPose2d pathPoint = states[bestPathPointIndex].pose;
  VexGpsPose2d nextPathPoint = states[bestPathPointIndex + 1].pose;
  return curState.CircleLineIntersectionPoint(m_maxLookAhead, pathPoint, nextPathPoint, tv);
}

} // namespace vpi