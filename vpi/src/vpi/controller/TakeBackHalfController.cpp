// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/log/Logger.h"
#include "vpi/controller/TakeBackHalfController.h"
#include "vpi/units/UnitUtils.h"

#include <algorithm>
#include <cmath>

using namespace vpi;

TakeBackHalfController::TakeBackHalfController(double g,
                              std::function<double()> measurementSource,
                              std::function<void(double)> useOutput,
                             QTime period)
    : m_G(g),
      m_measurementSource(std::move(measurementSource)),
      m_useOutput(std::move(useOutput)),
      m_period(period) {
  if (period < 5 * millisecond) {
    m_period = 5_ms;
    logger.log(Logger::LogLevel::WARN, "Controller period defaulted to 20ms");
  }
  static int instances = 0;
  instances++;
}

void TakeBackHalfController::SetGain(double g) {
  m_mutex.lock();
  m_G = g;
  m_mutex.unlock();
}

double TakeBackHalfController::GetGain() const {
  return m_G;
}

QTime TakeBackHalfController::GetPeriod() const {
  return m_period;
}

void TakeBackHalfController::SetSetpoint(double setpoint) {
  m_mutex.lock();
  m_setpoint = setpoint;
  m_mutex.unlock();
}

double TakeBackHalfController::GetSetpoint() const {
  return m_setpoint;
}

bool TakeBackHalfController::AtSetpoint() {
  return false; // Velocity Controllers are never really at their setpoint
}

double TakeBackHalfController::GetError() const {
  return m_Error;
}

double TakeBackHalfController::Calculate(double measurement) {
  m_mutex.lock();
  //double prev_measurement = m_measurement;
  m_measurement = measurement;
  m_prevError = m_Error;

  m_Error = m_setpoint - measurement;

  m_output += m_G * m_Error;
  if(sgn(m_Error) != sgn(m_prevError)) {
    m_output = .5 * (m_output + m_tbh);
    m_tbh = m_output;
  }
  m_mutex.unlock();
  return m_output;
}

void TakeBackHalfController::Reset() {
  m_mutex.lock();
  m_Error = 0;
  m_prevError = 0;
  m_output = 0;
  m_tbh = 0;
  m_FirstCalc = true;
  m_mutex.unlock();
}

void TakeBackHalfController::Disable() {
  // Task management
  if(m_controlTask != NULL) {
    m_controlTask->stop();
  }
  m_mutex.lock();
  m_enabled = false;
  m_mutex.unlock();
}

void TakeBackHalfController::Enable() {
  m_mutex.lock();
  m_enabled = true;
  m_mutex.unlock();
  // Task management
  if(m_controlTask != NULL) {
    m_controlTask->stop();
  }
  m_controlTask = new vex::task(TakeBackHalfController::_trampoline, static_cast<void *>(this));
}

void TakeBackHalfController::SetEnabled(bool enabled) {
  if(enabled) {
    if(m_measurementSource != NULL && m_useOutput != NULL) {
      Enable();
    } else {
      Disable();
    }
  } else {
    Disable();
  }
}

bool TakeBackHalfController::CheckEnabled()
{
  m_mutex.lock();
  bool retval = m_enabled;
  m_mutex.unlock();
  return retval;
}

void TakeBackHalfController::ControlLoop()
{
  while(1) {
    if(CheckEnabled()) {
      double m = m_measurementSource();
      m = Calculate(m);
      m_useOutput(m);
    }
    this_thread::sleep_for(m_period.convert(millisecond));
  }
}

int TakeBackHalfController::_trampoline(void *p_this) {
  TakeBackHalfController *p = (TakeBackHalfController *)p_this;
  while(1) {
    p->ControlLoop();
    // Sleep is handled in the ControlLoop function
  }

  return 0;
}
