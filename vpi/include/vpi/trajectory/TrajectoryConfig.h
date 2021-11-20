// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "vpi/kinematics/DifferentialDriveKinematics.h"
#include "vpi/kinematics/MecanumDriveKinematics.h"
#include "vpi/trajectory/CentripetalAccelerationConstraint.h"
#include "vpi/trajectory/DifferentialDriveKinematicsConstraint.h"
#include "vpi/trajectory/MecanumDriveKinematicsConstraint.h"
#include "vpi/trajectory/MaxVelocityConstraint.h"
#include "vpi/trajectory/TrajectoryConstraint.h"
#include "vpi/units/QAcceleration.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QSpeed.h"
#include "vpi/utils.h"

namespace vpi {
/**
 * Represents the configuration for generating a trajectory. This class stores
 * the start velocity, end velocity, max velocity, max acceleration, custom
 * constraints, and the reversed flag.
 *
 * The class must be constructed with a max velocity and max acceleration.
 * The other parameters (start velocity, end velocity, constraints, reversed)
 * have been defaulted to reasonable values (0, 0, {}, false). These values can
 * be changed via the SetXXX methods.
 */
class TrajectoryConfig {
 public:
  /**
   * Constructs a config object.
   * @param maxVelocity The max velocity of the trajectory.
   * @param maxAcceleration The max acceleration of the trajectory.
   */
  TrajectoryConfig(QSpeed maxVelocity,
                   QAcceleration maxAcceleration)
      : m_maxVelocity(maxVelocity), m_maxAcceleration(maxAcceleration) {}

  TrajectoryConfig(const TrajectoryConfig&) = delete;
  TrajectoryConfig& operator=(const TrajectoryConfig&) = delete;

  TrajectoryConfig(TrajectoryConfig&&) = default;
  TrajectoryConfig& operator=(TrajectoryConfig&&) = default;

  /**
   * Sets the start velocity of the trajectory.
   * @param startVelocity The start velocity of the trajectory.
   */
  void SetStartVelocity(QSpeed startVelocity) {
    m_startVelocity = startVelocity;
  }

  /**
   * Sets the end velocity of the trajectory.
   * @param endVelocity The end velocity of the trajectory.
   */
  void SetEndVelocity(QSpeed endVelocity) {
    m_endVelocity = endVelocity;
  }

  /**
   * Sets the reversed flag of the trajectory.
   * @param reversed Whether the trajectory should be reversed or not.
   */
  void SetReversed(bool reversed) { m_reversed = reversed; }

  /**
   * Adds a user-defined constraint to the trajectory.
   * @param constraint The user-defined constraint.
   */
  void AddConstraint(CentripetalAccelerationConstraint constraint) {
    m_constraints.emplace_back(std::make_unique<CentripetalAccelerationConstraint>(constraint));
  }
  void AddConstraint(DifferentialDriveKinematicsConstraint constraint) {
    m_constraints.emplace_back(std::make_unique<DifferentialDriveKinematicsConstraint>(constraint));
  }
  void AddConstraint(MecanumDriveKinematicsConstraint constraint) {
    m_constraints.emplace_back(std::make_unique<MecanumDriveKinematicsConstraint>(constraint));
  }
  void AddConstraint(MaxVelocityConstraint constraint) {
    m_constraints.emplace_back(std::make_unique<MaxVelocityConstraint>(constraint));
  }

  /**
   * Adds a differential drive kinematics constraint to ensure that
   * no wheel velocity of a differential drive goes above the max velocity.
   *
   * @param kinematics The differential drive kinematics.
   */
  void SetKinematics(const DifferentialDriveKinematics& kinematics) {
    AddConstraint(
        DifferentialDriveKinematicsConstraint(kinematics, m_maxVelocity));
  }

  /**
   * Adds a mecanum drive kinematics constraint to ensure that
   * no wheel velocity of a mecanum drive goes above the max velocity.
   *
   * @param kinematics The mecanum drive kinematics.
   */
  void SetKinematics(MecanumDriveKinematics kinematics) {
    AddConstraint(MecanumDriveKinematicsConstraint(kinematics, m_maxVelocity));
  }

  /**
   * Returns the starting velocity of the trajectory.
   * @return The starting velocity of the trajectory.
   */
  QSpeed StartVelocity() const { return m_startVelocity; }

  /**
   * Returns the ending velocity of the trajectory.
   * @return The ending velocity of the trajectory.
   */
  QSpeed EndVelocity() const { return m_endVelocity; }

  /**
   * Returns the maximum velocity of the trajectory.
   * @return The maximum velocity of the trajectory.
   */
  QSpeed MaxVelocity() const { return m_maxVelocity; }

  /**
   * Returns the maximum acceleration of the trajectory.
   * @return The maximum acceleration of the trajectory.
   */
  QAcceleration MaxAcceleration() const {
    return m_maxAcceleration;
  }

  /**
   * Returns the user-defined constraints of the trajectory.
   * @return The user-defined constraints of the trajectory.
   */
  const std::vector<std::unique_ptr<TrajectoryConstraint>>& Constraints()
      const {
    return m_constraints;
  }

  /**
   * Returns whether the trajectory is reversed or not.
   * @return whether the trajectory is reversed or not.
   */
  bool IsReversed() const { return m_reversed; }

 private:
  QSpeed m_startVelocity = 0_mps;
  QSpeed m_endVelocity = 0_mps;
  QSpeed m_maxVelocity;
  QAcceleration m_maxAcceleration;
  std::vector<std::unique_ptr<TrajectoryConstraint>> m_constraints;
  bool m_reversed = false;
};
}  // namespace vpi