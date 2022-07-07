// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <limits>

#include "vex.h"

#include "vpi/pid/PIDController.h"
#include "vpi/utils.h"
#include "vpi/units/QTime.h"

namespace vpi {

/**
 * Implements a PID control loop for velocity.
 */
class PIDVelocityController : public PIDController {
 public:
  PIDVelocityController(PIDFParameters p);

  /**
   * Allocates a PIDVelocityController with the given constants for Kp, Ki, Kd,
   * and Kf
   *
   * @param p      The PIDF coefficients.
   * @param measurementSource The function that returns the current state
   * @param useOutput The function that applies the result
   * @param period The period between controller updates in seconds. The
   *               default is 20 milliseconds. Must be non-zero and positive.
   */
  PIDVelocityController(PIDFParameters p,
                std::function<double()> measurementSource,
                std::function<void(double)> useOutput,
                QTime period = 20_ms) :
                PIDController(p, measurementSource, useOutput, period) {
    m_continuous = false;
  }

  ~PIDVelocityController() {
    Disable();
  }

  PIDVelocityController(const PIDVelocityController&) = default;
  PIDVelocityController& operator=(const PIDVelocityController&) = default;
  PIDVelocityController(PIDVelocityController&&) = default;
  PIDVelocityController& operator=(PIDVelocityController&&) = default;

  virtual bool AtSetpoint() override {return false;}
  virtual void DisableContinuousInput() override {}
  virtual void EnableContinuousInput(double minimumInput, double maximumInput) override {}
};

}  // namespace vpi
