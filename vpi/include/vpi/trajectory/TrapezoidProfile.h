// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <math.h>

#include "vpi/units/QTime.h"
#include "vpi/units/QLength.h"
#include "vpi/units/QSpeed.h"
#include "vpi/units/QAcceleration.h"

namespace vpi {

/**
 * A trapezoid-shaped velocity profile.
 *
 * While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on trapezoidal
 * velocity constraints. To compute the reference obeying this constraint, do
 * the following.
 *
 * Initialization:
 * @code{.cpp}
 * TrapezoidalMotionProfile::Constraints constraints{kMaxV, kMaxA};
 * double previousProfiledReference = initialReference;
 * @endcode
 *
 * Run on update:
 * @code{.cpp}
 * TrapezoidalMotionProfile profile{constraints, unprofiledReference,
 *                                  previousProfiledReference};
 * previousProfiledReference = profile.Calculate(timeSincePreviousUpdate);
 * @endcode
 *
 * where `unprofiledReference` is free to change between calls. Note that when
 * the unprofiled reference is within the constraints, `Calculate()` returns the
 * unprofiled reference unchanged.
 *
 * Otherwise, a timer can be started to provide monotonic values for
 * `Calculate()` and to determine when the profile has completed via
 * `IsFinished()`.
 */
class TrapezoidProfile {
 public:
  class Constraints {
   public:
    Constraints() {
    }
    Constraints(QSpeed maxVelocity_, QAcceleration maxAcceleration_)
        : maxVelocity{maxVelocity_}, maxAcceleration{maxAcceleration_} {
    }
    QSpeed maxVelocity = 0_mps;
    QAcceleration maxAcceleration = 0_mps2;
  };

  class State {
   public:
    State(QLength p, QSpeed v) : position(p), velocity(v) {}
    QLength position = 0_m;
    QSpeed velocity = 0_mps;
    bool operator==(const State& rhs) const {
      return position == rhs.position && velocity == rhs.velocity;
    }
    bool operator!=(const State& rhs) const { return !(*this == rhs); }
  };

  /**
   * Construct a TrapezoidProfile.
   *
   * @param constraints The constraints on the profile, like maximum velocity.
   * @param goal        The desired state when the profile is complete.
   * @param initial     The initial state (usually the current state).
   */
  TrapezoidProfile(Constraints constraints, State goal,
                   State initial = State{0_m, 0_mps});

  TrapezoidProfile(const TrapezoidProfile&) = default;
  TrapezoidProfile& operator=(const TrapezoidProfile&) = default;
  TrapezoidProfile(TrapezoidProfile&&) = default;
  TrapezoidProfile& operator=(TrapezoidProfile&&) = default;

  /**
   * Calculate the correct position and velocity for the profile at a time t
   * where the beginning of the profile was at time t = 0.
   *
   * @param t The time since the beginning of the profile.
   */
  State Calculate(QTime t) const;

  /**
   * Returns the time left until a target distance in the profile is reached.
   *
   * @param target The target distance.
   */
  QTime TimeLeftUntil(QLength target) const;

  /**
   * Returns the total time the profile takes to reach the goal.
   */
  QTime TotalTime() const { return m_endDeccel; }

  /**
   * Returns true if the profile has reached the goal.
   *
   * The profile has reached the goal if the time since the profile started
   * has exceeded the profile's total time.
   *
   * @param t The time since the beginning of the profile.
   */
  bool IsFinished(QTime t) const { return t >= TotalTime(); }

 private:
  /**
   * Returns true if the profile inverted.
   *
   * The profile is inverted if goal position is less than the initial position.
   *
   * @param initial The initial state (usually the current state).
   * @param goal    The desired state when the profile is complete.
   */
  static bool ShouldFlipAcceleration(const State& initial, const State& goal) {
    return initial.position > goal.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  State Direct(const State& in) const {
    State result = in;
    result.position *= m_direction;
    result.velocity *= m_direction;
    return result;
  }

  // The direction of the profile, either 1 for forwards or -1 for inverted
  int m_direction;

  Constraints m_constraints;
  State m_initial;
  State m_goal;

  QTime m_endAccel;
  QTime m_endFullSpeed;
  QTime m_endDeccel;
};
}  // namespace vpi

// #include "TrapezoidProfile.inc"
// Copied TrapezoidProfile.inc directly into the header...

namespace vpi {
TrapezoidProfile::TrapezoidProfile(Constraints constraints,
                                             State goal, State initial)
    : m_direction{ShouldFlipAcceleration(initial, goal) ? -1 : 1},
      m_constraints(constraints),
      m_initial(Direct(initial)),
      m_goal(Direct(goal)) {
  if (m_initial.velocity > m_constraints.maxVelocity) {
    m_initial.velocity = m_constraints.maxVelocity;
  }

  // Deal with a possibly truncated motion profile (with nonzero initial or
  // final velocity) by calculating the parameters as if the profile began and
  // ended at zero velocity
  QTime cutoffBegin =
      m_initial.velocity / m_constraints.maxAcceleration;
  QLength cutoffDistBegin =
      cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

  QTime cutoffEnd = m_goal.velocity / m_constraints.maxAcceleration;
  QLength cutoffDistEnd =
      cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

  // Now we can calculate the parameters as if it was a full trapezoid instead
  // of a truncated one

  QLength fullTrapezoidDist =
      cutoffDistBegin + (m_goal.position - m_initial.position) + cutoffDistEnd;
  QTime accelerationTime =
      m_constraints.maxVelocity / m_constraints.maxAcceleration;

  QLength fullSpeedDist =
      fullTrapezoidDist -
      accelerationTime * accelerationTime * m_constraints.maxAcceleration;

  // Handle the case where the profile never reaches full speed
  if (fullSpeedDist < 0_m) {
    accelerationTime = sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
    fullSpeedDist = 0_m;
  }

  m_endAccel = accelerationTime - cutoffBegin;
  m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
  m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
}

TrapezoidProfile::State TrapezoidProfile::Calculate(QTime t) const {
  State result = m_initial;

  if (t < m_endAccel) {
    result.velocity += t * m_constraints.maxAcceleration;
    result.position +=
        (m_initial.velocity + t * m_constraints.maxAcceleration / 2.0) * t;
  } else if (t < m_endFullSpeed) {
    result.velocity = m_constraints.maxVelocity;
    result.position += (m_initial.velocity +
                        m_endAccel * m_constraints.maxAcceleration / 2.0) *
                           m_endAccel +
                       m_constraints.maxVelocity * (t - m_endAccel);
  } else if (t <= m_endDeccel) {
    result.velocity =
        m_goal.velocity + (m_endDeccel - t) * m_constraints.maxAcceleration;
    QTime timeLeft = m_endDeccel - t;
    result.position =
        m_goal.position -
        (m_goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) *
            timeLeft;
  } else {
    result = m_goal;
  }

  return Direct(result);
}

QTime TrapezoidProfile::TimeLeftUntil(
    QLength target) const {
  QLength position = m_initial.position * m_direction;
  QSpeed velocity = m_initial.velocity * m_direction;

  QTime endAccel = m_endAccel * m_direction;
  QTime endFullSpeed = m_endFullSpeed * m_direction - endAccel;

  if (target < position) {
    endAccel *= -1.0;
    endFullSpeed *= -1.0;
    velocity *= -1.0;
  }

  endAccel = fmax(endAccel.convert(second), 0) * second;
  endFullSpeed = fmax(endFullSpeed.convert(second), 0) * second;
  QTime endDeccel = m_endDeccel - endAccel - endFullSpeed;
  endDeccel = fmax(endDeccel.convert(second), 0) * second;

  const QAcceleration acceleration = m_constraints.maxAcceleration;
  const QAcceleration decceleration = -1.0 * m_constraints.maxAcceleration;

  QLength distToTarget = fabs(target.convert(meter) - position.convert(meter)) * meter;

  if (distToTarget < 1e-6 * meter) {
    return 0_s;
  }

  QLength accelDist =
      velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

  QSpeed deccelVelocity;
  if (endAccel > 0_s) {
    deccelVelocity = sqrt(abs(velocity * velocity + 2 * acceleration * accelDist));
  } else {
    deccelVelocity = velocity;
  }

  QLength deccelDist =
      deccelVelocity * endDeccel + 0.5 * decceleration * endDeccel * endDeccel;

  deccelDist = max(deccelDist, 0_m);

  QLength fullSpeedDist = m_constraints.maxVelocity * endFullSpeed;

  if (accelDist > distToTarget) {
    accelDist = distToTarget;
    fullSpeedDist = 0_m;
    deccelDist = 0_m;
  } else if (accelDist + fullSpeedDist > distToTarget) {
    fullSpeedDist = distToTarget - accelDist;
    deccelDist = 0_m;
  } else {
    deccelDist = distToTarget - fullSpeedDist - accelDist;
  }

  QTime accelTime = second *
      (-velocity.convert(mps) + 
              std::sqrt(fabs(velocity.convert(mps) * velocity.convert(mps) + 
                            2 * acceleration.convert(mps2) * accelDist.convert(meter)))) /
      acceleration.convert(mps2);

  QTime deccelTime = second *
      (-deccelVelocity.convert(mps) +
       std::sqrt(fabs(deccelVelocity.convert(mps) * deccelVelocity.convert(mps) +
                                          2 * decceleration.convert(mps2) * deccelDist.convert(meter)))) /
      decceleration.convert(mps2);

  QTime fullSpeedTime = fullSpeedDist / m_constraints.maxVelocity;

  return accelTime + fullSpeedTime + deccelTime;
}
}  // namespace vpi