// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/controller/BangBangController.h"
#include "vpi/log/Logger.h"
#include "vpi/units/UnitUtils.h"

using namespace vpi;

BangBangController::BangBangController(std::function<double()> measurementSource,
                                        std::function<void(double)> useOutput,
                                        QTime period,
                                        double tolerance)
    : m_measurementSource(std::move(measurementSource)),
      m_useOutput(std::move(useOutput)),
      m_period(period),
      m_tolerance(tolerance) {
  if (period <= 5 * millisecond) {
    m_period = 20_ms;
    logger.log(Logger::LogLevel::WARN, "Controller period defaulted to 20ms");
  }
  static int instances = 0;
  instances++;
}

void BangBangController::SetSetpoint(double setpoint) {
  m_setpoint = setpoint;
}

double BangBangController::GetSetpoint() const {
  return m_setpoint;
}

bool BangBangController::AtSetpoint() const {
  return std::abs(m_setpoint - m_measurement) < m_tolerance;
}

void BangBangController::SetTolerance(double tolerance) {
  m_tolerance = tolerance;
}

double BangBangController::GetTolerance() const {
  return m_tolerance;
}

double BangBangController::GetMeasurement() const {
  return m_measurement;
}

double BangBangController::GetError() const {
  return m_setpoint - m_measurement;
}

double BangBangController::Calculate(double measurement, double setpoint) {
  m_measurement = measurement;
  m_setpoint = setpoint;

  return measurement < setpoint ? 1 : 0;
}

double BangBangController::Calculate(double measurement) {
  return Calculate(measurement, m_setpoint);
}

void BangBangController::Disable() {
  // Task management
  if(m_controlTask != NULL) {
    m_controlTask->stop();
  }
  m_mutex.lock();
  m_enabled = false;
  m_mutex.unlock();
}

void BangBangController::Enable() {
  m_mutex.lock();
  m_enabled = true;
  m_mutex.unlock();
  // Task management
  if(m_controlTask != NULL) {
    m_controlTask->stop();
  }
  m_controlTask = new vex::task(BangBangController::_trampoline, static_cast<void *>(this));
}

void BangBangController::SetEnabled(bool enabled) {
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

bool BangBangController::CheckEnabled()
{
  m_mutex.lock();
  bool retval = m_enabled;
  m_mutex.unlock();
  return retval;
}

void BangBangController::ControlLoop()
{
  while(CheckEnabled()) {
    double m = m_measurementSource();
    m = Calculate(m);
    m_useOutput(m);
    this_thread::sleep_for(m_period.convert(millisecond));
  }
}

int BangBangController::_trampoline(void *p_this) {
  BangBangController *p = (BangBangController *)p_this;
  while(1) {
    p->ControlLoop();
    // Sleep is handled in the ControlLoop function
  }

  return 0;
}
