// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <limits>

#include "vex.h"

#include "vpi/utils.h"
#include "vpi/units/QTime.h"

namespace vpi {

/**
 * Take Back Half controller
 *
 * * https://www.vexwiki.org/programming/controls_algorithms/tbh
 *
 * This Controller makes use of some more advanced concepts:
 * * Pointers
 * * Function Pointers
 * * Tasks
 * * Mutex
 *
 * Unfortunately, to create an instance of this class, one will
 * need to understand Function Pointers, which can be difficult
 * to understand for new programmers.  A good primer would be
 * to read:
 *
 *    https://www.learncpp.com/cpp-tutorial/function-pointers/
 *
 * The basic concept is that this Controller allows the user
 * to specify their own functions to provide the "Measurement
 * Source" and the Output.  The measurementSource function
 * *must* return a double and take no arguments.  The function
 * to apply the output *must* return void and take a double as
 * its sole argument.
 */
class TakeBackHalfController {
 public:
  /**
   * Allocates a PIDController with the given constants for Kp, Ki, Kd,
   * and Kf
   *
   * @param g      The gain to apply
   * @param measurementSource The function that returns the current state
   * @param useOutput The function that applies the result
   * @param period The period between controller updates in seconds. The
   *               default is 10 milliseconds. Must be non-zero and positive.
   */
  TakeBackHalfController(double g,
                std::function<double()> measurementSource,
                std::function<void(double)> useOutput,
                QTime period = 10_ms);

  ~TakeBackHalfController() {
    Disable();
  }

  TakeBackHalfController(const TakeBackHalfController&) = default;
  TakeBackHalfController& operator=(const TakeBackHalfController&) = default;
  TakeBackHalfController(TakeBackHalfController&&) = default;
  TakeBackHalfController& operator=(TakeBackHalfController&&) = default;

  /**
   * Sets the gain parameter.
   *
   * @param g  Gain to use.
   */
  void SetGain(double g);

  /**
   * Gets the gain parameter.
   */
  double GetGain() const;

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  QTime GetPeriod() const;

  /**
   * Sets the setpoint for the PIDController.
   *
   * @param setpoint The desired setpoint.
   */
  void SetSetpoint(double setpoint);

  /**
   * Returns the current setpoint of the PIDController.
   *
   * @return The current setpoint.
   */
  double GetSetpoint() const;

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * This will return false until at least one input value has been computed.
   */
  virtual bool AtSetpoint();

  /**
   * Returns the difference between the setpoint and the measurement.
   */
  double GetError() const;

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  double Calculate(double measurement);

  /**
   * Reset the previous error, the integral term, and disable the controller.
   */
  void Reset();

  bool IsEnabled() {
    return CheckEnabled();
  }

  void SetEnabled(bool enable);

  void Enable();
  void Disable();
  bool CheckEnabled();

 protected:
  // Factors for PIDF controls
  double m_G;

  std::function<double()> m_measurementSource = NULL;
  std::function<void(double)> m_useOutput = NULL;

  // The period (in seconds) of the control loop running this controller
  QTime m_period;

  // The error at the time of the most recent call to Calculate()
  double m_Error = 0;

  // The error at the time of the second-most-recent call to Calculate() (used
  // to compute velocity)
  double m_prevError = 0;

  double m_output = 0;
  double m_tbh = 0;

  double m_setpoint = 0;
  double m_measurement = 0;

  vex::mutex m_mutex;

  bool m_enabled = false;
  bool m_FirstCalc = true;

  private:
  vex::task *m_controlTask = NULL;

  void ControlLoop();
  static int _trampoline(void *p_this);

};

}  // namespace vpi
