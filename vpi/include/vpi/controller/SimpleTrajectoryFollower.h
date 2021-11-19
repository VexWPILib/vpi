// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/trajectory/Trajectory.h"
#include "vpi/units/QTime.h"

namespace vpi {
  /**
   * For each point, simply turns to face the next point, then drives to it
   * stopping at each point.
   */
class SimpleTrajectoryFollower {
  public:
    SimpleTrajectoryFollower(DifferentialDriveChassis *chassis) :
        m_chassis(chassis) {}

    void FollowTrajectory(std::vector<Trajectory::State> states);
    void FollowTrajectory(std::initializer_list<Trajectory::State> states){
      std::vector<Trajectory::State> vstates = states;
      FollowTrajectory(vstates);
    }
  protected:
    DifferentialDriveChassis *m_chassis;
};
} // vpi
