// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include "vpi/geometry/Pose2d.h"
#include "vpi/geometry/VexGpsPose2d.h"
#include "vpi/geometry/Transform2d.h"
#include "vpi/units/QAcceleration.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QTime.h"
#include "vpi/units/QSpeed.h"

namespace vpi {
/**
 * Represents a time-parameterized trajectory. The trajectory contains of
 * various States that represent the pose, curvature, time elapsed, velocity,
 * and acceleration at that point.
 */
class Trajectory {
 public:
  /**
   * Represents one point on the trajectory.
   */
  struct State {
    // The time elapsed since the beginning of the trajectory.
    QTime t = 0_s;

    // The speed at that point of the trajectory.
    QSpeed velocity = 0_mps;

    // The acceleration at that point of the trajectory.
    QAcceleration acceleration = 0_mps2;

    // The pose at that point of the trajectory.
    Pose2d pose = Pose2d({0_m, 0_m}, 0_deg);

    // The curvature at that point of the trajectory.
    QCurvature curvature = 0.0_radpermeter;

    /**
     * Checks equality between this State and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are equal.
     */
    bool operator==(const State& other) const;

    /**
     * Checks inequality between this State and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are not equal.
     */
    bool operator!=(const State& other) const;

    /**
     * Interpolates between two States.
     *
     * @param endValue The end value for the interpolation.
     * @param i The interpolant (fraction).
     *
     * @return The interpolated state.
     */
    State Interpolate(State endValue, double i) const;

    State(QTime ta, QSpeed v, QAcceleration a, Pose2d p, QCurvature c) :
      t(ta), velocity(v), acceleration(a), pose(p), curvature(c)
    {}

    State() {}
  };

  Trajectory() = default;

  /**
   * Constructs a trajectory from a vector of states.
   */
  explicit Trajectory(const std::vector<State>& states);

  /**
   * Returns the overall duration of the trajectory.
   * @return The duration of the trajectory.
   */
  QTime TotalTime() const { return m_totalTime; }

  /**
   * Return the states of the trajectory.
   * @return The states of the trajectory.
   */
  const std::vector<State>& States() const { return m_states; }

  /**
   * Sample the trajectory at a point in time.
   *
   * @param t The point in time since the beginning of the trajectory to sample.
   * @return The state at that point in time.
   */
  State Sample(QTime t) const;

  /**
   * Transforms all poses in the trajectory by the given transform. This is
   * useful for converting a robot-relative trajectory into a field-relative
   * trajectory. This works with respect to the first pose in the trajectory.
   *
   * @param transform The transform to transform the trajectory by.
   * @return The transformed trajectory.
   */
  Trajectory TransformBy(const Transform2d& transform);

  /**
   * Transforms all poses in the trajectory so that they are relative to the
   * given pose. This is useful for converting a field-relative trajectory
   * into a robot-relative trajectory.
   *
   * @param pose The pose that is the origin of the coordinate frame that
   *             the current trajectory will be transformed into.
   * @return The transformed trajectory.
   */
  Trajectory RelativeTo(const Pose2d& pose);

  /**
   * Concatenates another trajectory to the current trajectory. The user is
   * responsible for making sure that the end pose of this trajectory and the
   * start pose of the other trajectory match (if that is the desired behavior).
   *
   * @param other The trajectory to concatenate.
   * @return The concatenated trajectory.
   */
  Trajectory operator+(const Trajectory& other) const;

  /**
   * Returns the initial pose of the trajectory.
   *
   * @return The initial pose of the trajectory.
   */
  VexGpsPose2d InitialPose() const { return Sample(0_s).pose; }

  /**
   * Checks equality between this Trajectory and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Trajectory& other) const;

  /**
   * Checks inequality between this Trajectory and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are inequal.
   */
  bool operator!=(const Trajectory& other) const;

 private:
  std::vector<State> m_states;
  QTime m_totalTime = 0_s;

  /**
   * Linearly interpolates between two values.
   *
   * @param startValue The start value.
   * @param endValue The end value.
   * @param t The fraction for interpolation.
   *
   * @return The interpolated value.
   */
  template <typename T>
  static T Lerp(const T& startValue, const T& endValue, const double t) {
    return startValue + (endValue - startValue) * t;
  }
};

}  // namespace vpi