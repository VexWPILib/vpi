// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>

#include "vpi/units/QAcceleration.h"
#include "vpi/units/QFrequency.h"
#include "vpi/units/QSpeed.h"
#include "vpi/units/QTime.h"
#include "vpi/units/UnitUtils.h"
#include "vpi/utils.h"

namespace vpi {
// TODO - Figure out how to do this with templates, and then merge with AngularSlewRateLimiter
/**
 * A class that limits the rate of change of an input value.  Useful for
 * implementing voltage, setpoint, and/or output ramps.  A slew-rate limit
 * is most appropriate when the quantity being controlled is a velocity or
 * a voltage; when controlling a position, consider using a TrapezoidProfile
 * instead.
 *
 * @see TrapezoidProfile
 */
class LinearSlewRateLimiter {
 public:

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit.
   * @param initialValue The initial value of the input.
   */
  explicit LinearSlewRateLimiter(QAcceleration rateLimit, QSpeed initialValue = 0_mps)
      : m_rateLimit{rateLimit},
        m_prevVal{initialValue},
        m_prevTime{UnitUtils::now()} {}

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew
   * rate.
   */
  QSpeed Calculate(QSpeed input) {
    QTime currentTime = UnitUtils::now();
    QTime elapsedTime = currentTime - m_prevTime;
    m_prevVal += VpiUtils::clip(input - m_prevVal, -m_rateLimit * elapsedTime,
                            m_rateLimit * elapsedTime);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit
   * when doing so.
   *
   * @param value The value to reset to.
   */
  void Reset(QSpeed value) {
    m_prevVal = value;
    m_prevTime = UnitUtils::now();
  }

 private:
  QAcceleration m_rateLimit;
  QSpeed m_prevVal;
  QTime m_prevTime;
};
}  // namespace vpi
