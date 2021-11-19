// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <thread>

#include "vex.h"
#include "vpi/hal/motor/AngularSlewRateLimitedMotor.h"
#include "vpi/units/QAcceleration.h"
#include "vpi/units/QSpeed.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {
  /**
   * Example usage:
   *
   * @code{.cpp}
   * vex::motor ml = motor(PORT1, ratio18_1, false);  // also works for vex::motor_group
   * LinearSlewRateLimitedMotor lsrlm(ml, 0.5_mps2, 3.25_in);
   * lsrlm.SetLinearSpeed(2_mps);
   * wait(5, vex::seconds);
   * lsrlm.SetLinearSpeed(0_mps);
   * wait(5, vex::seconds);
   * @endcode
   * 
   */
  class LinearSlewRateLimitedMotor {
    public:
      LinearSlewRateLimitedMotor(vex::motor m, QAcceleration r, QLength diameter, double gearRatio = 1.0) 
          : m_acc(r), m_diameter(diameter)
      {
        m_gearRatio = gearRatio;
        createAngularSlewRateLimiter({m});
      }

      LinearSlewRateLimitedMotor(vex::motor_group mg, QAcceleration r, QLength diameter, double gearRatio = 1.0) 
          : m_acc(r), m_diameter(diameter)
      {
        m_gearRatio = gearRatio;
        createAngularSlewRateLimiter(mg);
      }

      void SetLinearSpeed(QSpeed s) {
        if(m_asrlm != NULL) {
          QAngularSpeed sa = UnitUtils::convertLinearSpeedToRotationalSpeed(s, m_diameter, m_gearRatio);
          m_asrlm->SetAngularSpeed(sa);
        }
      }

    protected:
      AngularSlewRateLimitedMotor *m_asrlm = NULL;
      QAcceleration m_acc;
      QLength m_diameter;
      double m_gearRatio;

    private:
      void createAngularSlewRateLimiter(vex::motor_group mg) {
        QSpeed s = m_acc * second;
        QAngularSpeed sa = UnitUtils::convertLinearSpeedToRotationalSpeed(s, m_diameter, m_gearRatio);
        m_asrlm = new AngularSlewRateLimitedMotor(mg, sa / second);
      }
  };
} // vpi
