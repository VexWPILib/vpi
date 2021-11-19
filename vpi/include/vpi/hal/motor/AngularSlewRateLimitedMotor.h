// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <thread>

#include "vex.h"
#include "vpi/filter/AngularSlewRateLimiter.h"

namespace vpi {
  /**
   * Example usage:
   *
   * @code{.cpp}
   * vex::motor ml = motor(PORT1, ratio18_1, false);  // also works for vex::motor_group
   * AngularSlewRateLimitedMotor asrlm(ml, 50_rpm / second);
   * asrlm.SetAngularSpeed(200_rpm);
   * wait(5, vex::seconds);
   * asrlm.SetAngularSpeed(0_rpm);
   * wait(5, vex::seconds);
   * @endcode
   * 
   */
  class AngularSlewRateLimitedMotor {
    public:
      AngularSlewRateLimitedMotor(vex::motor m, QAngularAcceleration r) 
          : m_mg{m}, m_asrl(r), m_angularspeed(0_radps) 
      {
        m_rateMonitorTask = new vex::task(AngularSlewRateLimitedMotor::_trampoline, static_cast<void *>(this));
      }

      AngularSlewRateLimitedMotor(vex::motor_group mg, QAngularAcceleration r) 
          : m_mg(mg), m_asrl(r), m_angularspeed(0_radps) 
      {
        m_rateMonitorTask = new vex::task(AngularSlewRateLimitedMotor::_trampoline, static_cast<void *>(this));
      }

      void SetAngularSpeed(QAngularSpeed s) {
        m_angularspeed = s;
      }

    protected:
      vex::motor_group m_mg;
      AngularSlewRateLimiter m_asrl;
      QAngularSpeed m_angularspeed;

    private:
      vex::task *m_rateMonitorTask = NULL;
      void UpdateMotorSpeed() {
        QAngularSpeed actual = m_asrl.Calculate(m_angularspeed);
        m_mg.spin(vex::fwd,actual.convert(rpm),vex::rpm);
      }

      static int _trampoline(void *p_this) {
        AngularSlewRateLimitedMotor *aslrlm = (AngularSlewRateLimitedMotor *)p_this;
        while(1) {
          aslrlm->UpdateMotorSpeed();

          this_thread::sleep_for(20);
        }

        return 0;
      }

  };
} // namespace vpi
