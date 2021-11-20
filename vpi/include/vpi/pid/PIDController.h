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

#include "vpi/utils.h"
#include "vpi/units/QTime.h"

namespace vpi {

struct PIDFParameters {
  /**
   * Allocates a PID Parameters with the given constants for Kp, Ki, Kd,
   * and Kf
   *
   * @param Kp     The proportional coefficient.
   * @param Ki     The integral coefficient.
   * @param Kd     The derivative coefficient.
   * @param Kf     The feedforward coefficient.
   */
  PIDFParameters(double p, double i, double d, double f=0) {
    Kp = p;
    Ki = i;
    Kd = d;
    Kf = f;
  }

  double Kp;
  double Ki;
  double Kd;
  double Kf;
};

/**
 * Implements a PID control loop.
 *
 * To understand what a PID control loop is, read:
 *
 * * https://www.ni.com/en-us/innovations/white-papers/06/pid-theory-explained.html
 * * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html#pid-control-in-wpilib
 * * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#feedforward-control-in-wpilib
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
 *
 * An example of use is in the DifferentialDriveChassisPID class:
 * 
 * @code{.cpp}
 * p = new PIDController(pParams, [this] {return this->DistanceToTarget().convert(inch);},
 *                       [this](double v) {this->ConsumeDistancePID(v);});
 * @endcode
 *
 * The pParams should be self-explanatory, it is a structure for holding
 * the PIDF coefficients.
 *
 * The second and third arguments are similar. Both are "lambdas" -
 * executable snippets of code.  The first takes no arguments and
 * implicitly returns a double (since the .convert(inch) portion returns
 * a double).  The `[this]` portion means that a pointer to the
 * current object instance `this` is available inside the lambda, which
 * allows us to call this->DistanceToTarget() (and then .convert(inch) on
 * the result)
 *
 * The third argument is similar to the second, but this time the
 * `(double v)` tells us that this lambda will take one parameter
 * of type double as an argument.  As before `[this]` gives us access
 * to a pointer to the current instance of the object inside the lambda,
 * which allows us to pass the value `v` into `ConsumeDistancePID`
 *
 * When the PIDController instance's SetEnabled is called with the value
 * `true` as the argument, a `task` is created, basically a separate
 * thread of execution.  The V5 Brain makes it seem like more than 1
 * set of instructions are executing at the same time.  In this case,
 * SetEnabled will then return control flow to whatever line after it
 * was called while the `task` runs simultaneously.  This task calls
 * this class's `_trampoline` function, which then calls the `ControlLoop`
 * method.
 * 
 * Inside the ControlLoop, we see that we call the function pointed to
 * by the `m_measurementSource` property, stores that in the variable
 * `m` which is then passed into the function pointed to by the property
 * `m_useOutput`
 */
class PIDController {
 public:
  PIDController(PIDFParameters p);

  /**
   * Allocates a PIDController with the given constants for Kp, Ki, Kd,
   * and Kf
   *
   * @param p      The PIDF coefficients.
   * @param measurementSource The function that returns the current state
   * @param useOutput The function that applies the result
   * @param period The period between controller updates in seconds. The
   *               default is 20 milliseconds. Must be non-zero and positive.
   */
  PIDController(PIDFParameters p,
                std::function<double()> measurementSource,
                std::function<void(double)> useOutput,
                QTime period = 20_ms);

  ~PIDController() = default;

  PIDController(const PIDController&) = default;
  PIDController& operator=(const PIDController&) = default;
  PIDController(PIDController&&) = default;
  PIDController& operator=(PIDController&&) = default;

  /**
   * Sets the PID Controller gain parameters.
   *
   * Sets the proportional, integral, and differential coefficients.
   *
   * @param p  PIDF coefficients.
   */
  void SetPID(PIDFParameters p);

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  double GetP() const;

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  double GetI() const;

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  double GetD() const;

  /**
   * Gets the feedforward coefficient.
   *
   * @return feedforward coefficient
   */
  double GetF() const;

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
  bool AtSetpoint() const;

  /**
   * Enables continuous input.
   *
   * Rather then using the max and min input range as constraints, it considers
   * them to be the same point and automatically calculates the shortest route
   * to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  void EnableContinuousInput(double minimumInput, double maximumInput);

  /**
   * Disables continuous input.
   */
  void DisableContinuousInput();

  /**
   * Returns true if continuous input is enabled.
   */
  bool IsContinuousInputEnabled() const;

  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * When the cap is reached, the integrator value is added to the controller
   * output rather than the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  void SetIntegratorRange(double minimumIntegral, double maximumIntegral);

  /**
   * Sets the error which is considered tolerable for use with AtSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  void SetTolerance(
      double positionTolerance,
      double velocityTolerance = std::numeric_limits<double>::infinity());

  /**
   * Returns the difference between the setpoint and the measurement.
   */
  double GetPositionError() const;

  /**
   * Returns the velocity error.
   */
  double GetVelocityError() const;

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

 private:
  // Factors for PIDF controls
  PIDFParameters m_P;

  std::function<double()> m_measurementSource = NULL;
  std::function<void(double)> m_useOutput = NULL;

  // The period (in seconds) of the control loop running this controller
  QTime m_period;

  double m_maximumIntegral = 1.0;

  double m_minimumIntegral = -1.0;

  double m_maximumInput = 0;

  double m_minimumInput = 0;

  // Do the endpoints wrap around? eg. Absolute encoder
  bool m_continuous = false;

  // The error at the time of the most recent call to Calculate()
  double m_positionError = 0;
  double m_velocityError = 0;

  // The error at the time of the second-most-recent call to Calculate() (used
  // to compute velocity)
  double m_prevError = 0;

  // The sum of the errors for use in the integral calc
  double m_totalError = 0;

  // The error that is considered at setpoint.
  double m_positionTolerance = 0.05;
  double m_velocityTolerance = std::numeric_limits<double>::infinity();

  double m_setpoint = 0;
  double m_measurement = 0;

  vex::mutex m_mutex;

  bool m_enabled = false;

  void Enable();
  void Disable();
  bool CheckEnabled();

  vex::task *m_controlTask = NULL;

  void ControlLoop();
  static int _trampoline(void *p_this);

};

}  // namespace vpi
