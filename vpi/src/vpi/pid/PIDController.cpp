// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/pid/PIDController.h"

#include <algorithm>
#include <cmath>

using namespace vpi;

PIDController::PIDController(PIDFParameters p)
    : m_P(p),  m_period(20_ms) {
  static int instances = 0;
  instances++;
}

PIDController::PIDController(PIDFParameters p,
                              std::function<double()> measurementSource,
                              std::function<void(double)> useOutput,
                             QTime period)
    : m_P(p),
      m_measurementSource(std::move(measurementSource)),
      m_useOutput(std::move(useOutput)),
      m_period(period) {
  if (period <= 5 * millisecond) {
    /* ReportError(
        "Controller period must be a non-zero positive number, got {}!",
        period.value());*/
    m_period = 20_ms;
    /*ReportWarning(
        "{}", "Controller period defaulted to 20ms.");*/
  }
  static int instances = 0;
  instances++;
}

void PIDController::SetPID(PIDFParameters p) {
  m_mutex.lock();
  m_P = p;
  m_mutex.unlock();
}

double PIDController::GetP() const {
  return m_P.Kp;
}

double PIDController::GetI() const {
  return m_P.Ki;
}

double PIDController::GetD() const {
  return m_P.Kd;
}

double PIDController::GetF() const {
  return m_P.Kf;
}

QTime PIDController::GetPeriod() const {
  return m_period;
}

void PIDController::SetSetpoint(double setpoint) {
  m_mutex.lock();
  m_setpoint = setpoint;
  m_mutex.unlock();
}

double PIDController::GetSetpoint() const {
  return m_setpoint;
}

bool PIDController::AtSetpoint() const {
  double positionError;
  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    positionError =
        VpiUtils::clip(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    positionError = m_setpoint - m_measurement;
  }

  double velocityError = (positionError - m_prevError) / m_period.convert(second);

  return std::abs(positionError) < m_positionTolerance &&
         std::abs(velocityError) < m_velocityTolerance;
}

void PIDController::EnableContinuousInput(double minimumInput,
                                          double maximumInput) {
  m_mutex.lock();                                            
  m_continuous = true;
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
  m_mutex.unlock();
}

void PIDController::DisableContinuousInput() {
  m_mutex.lock();
  m_continuous = false;
  m_mutex.unlock();
}

bool PIDController::IsContinuousInputEnabled() const {
  return m_continuous;
}

void PIDController::SetIntegratorRange(double minimumIntegral,
                                       double maximumIntegral) {
  m_mutex.lock();
  m_minimumIntegral = minimumIntegral;
  m_maximumIntegral = maximumIntegral;
  m_mutex.unlock();
}

void PIDController::SetTolerance(double positionTolerance,
                                 double velocityTolerance) {
  m_mutex.lock();
  m_positionTolerance = positionTolerance;
  m_velocityTolerance = velocityTolerance;
  m_mutex.unlock();
}

double PIDController::GetPositionError() const {
  return m_positionError;
}

double PIDController::GetVelocityError() const {
  return m_velocityError;
}

double PIDController::Calculate(double measurement) {
  m_mutex.lock();
  double prev_measurement = m_measurement;
  m_measurement = measurement;
  m_prevError = m_positionError;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError =
        VpiUtils::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period.convert(second);

  if (m_P.Ki != 0) {
    m_totalError =
        VpiUtils::clip(m_totalError + m_positionError * m_period.convert(second),
                   m_minimumIntegral / m_P.Ki, m_maximumIntegral / m_P.Ki);
  }

  double retval = m_P.Kp * m_positionError + m_P.Ki * m_totalError + m_P.Kd * m_velocityError + 
        m_P.Kf * (prev_measurement - measurement);
  m_mutex.unlock();
  return retval;
}

void PIDController::Reset() {
  Disable();

  m_mutex.lock();
  m_prevError = 0;
  m_totalError = 0;
  m_mutex.unlock();
}

void PIDController::Disable() {
  // Task management
  if(m_controlTask != NULL) {
    m_controlTask->stop();
  }
  m_mutex.lock();
  m_enabled = false;
  m_mutex.unlock();
}

void PIDController::Enable() {
  m_mutex.lock();
  m_enabled = true;
  m_mutex.unlock();
  // Task management
  if(m_controlTask != NULL) {
    m_controlTask->stop();
  }
  m_controlTask = new vex::task(PIDController::_trampoline, static_cast<void *>(this));
}

void PIDController::SetEnabled(bool enabled) {
  if(enabled) {
    if(m_measurementSource != NULL && m_useOutput != NULL) {
      Enable();
    }
  } else {
    Disable();
  }
}

bool PIDController::CheckEnabled()
{
  m_mutex.lock();
  bool retval = m_enabled;
  m_mutex.unlock();
  return retval;
}

void PIDController::ControlLoop()
{
  while(!AtSetpoint() && CheckEnabled()) {
    double m = m_measurementSource();
    m_useOutput(m);
    this_thread::sleep_for(m_period.convert(millisecond));
  }
}

int PIDController::_trampoline(void *p_this) {
  PIDController *p = (PIDController *)p_this;
  while(1) {
    p->ControlLoop();
    // Sleep is handled in the ControlLoop function
  }

  return 0;
}
