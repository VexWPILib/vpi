// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <thread>

#include "vex.h"
#include "vpi/log/Logger.h"
#include "vpi/controller/BangBangController.h"
#include "vpi/units/QAngularSpeed.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {
  /**
   * Example usage (see warnings about BangBangController):
   *
   * @code{.cpp}
   * vex::motor m = motor(PORT1, ratio18_1, false);  // also works for vex::motor_group
   * // Should go from RPM to Volts for scaling, so p might be 12 (V) / 200 (rpm)
   * // for an ungeared green-cartridge motor
   * VelocityBangBangMotor vm(ml, ratio18_1, 84.0 / 12.0);
   * vm.SetAngularSpeed(100_rpm);
   * wait(5, vex::seconds);
   * vm.SetAngularSpeed(200_rpm);
   * wait(5, vex::seconds);
   * @endcode
   * 
   */
  class VelocityBangBangMotor {
    public:
      VelocityBangBangMotor(vex::motor m, vex::gearSetting gs, double gearRatio = 1.0) 
          : m_mg({m}), m_gs(gs)
      {
        m_gearRatio = gearRatio;
      }

      VelocityBangBangMotor(vex::motor_group mg, vex::gearSetting gs, double gearRatio = 1.0) 
          : m_mg(mg), m_gs(gs)
      {
        m_gearRatio = gearRatio;
        m_bangBangController = new BangBangController([this] {return this->GetCurrentAngularSpeed().convert(rpm);},
                                              [this](double v) {this->ConsumeAngularSpeed(v);});
      }

      virtual ~VelocityBangBangMotor() {
        Disable();
        if(m_bangBangController != NULL) {
          m_bangBangController->Disable();
          m_bangBangController = NULL;
        }
      }

      void SetAngularSpeed(QAngularSpeed s) {
        m_mutex.lock();
        m_angularspeed_target = s;
        m_mutex.unlock();

        if(m_bangBangController != NULL) {
          m_bangBangController->SetSetpoint(m_angularspeed_target.convert(rpm));
        }        
      }

      void Enable() {
        if(m_bangBangController != NULL) {
          m_bangBangController->Enable();
        }
      }
      
      void Disable() {
        if(m_bangBangController != NULL) {
          m_bangBangController->Disable();
        }        
      }

      bool IsEnabled() {
        if(m_bangBangController != NULL) {
          return m_bangBangController->CheckEnabled();
        }
        return false;
      }

      void Reset() {
        if(m_bangBangController != NULL) {
          m_bangBangController->Reset();
        }
      }

      virtual QAngularSpeed GetCurrentAngularSpeed() {
        m_mutex.lock();
        double motorRpm = m_mg.velocity(vex::velocityUnits::rpm);
        QAngularSpeed curRpm = motorRpm * m_gearRatio * rpm;
        m_mutex.unlock();
        return curRpm;
      }

    protected:
      vex::motor_group m_mg;
      vex::gearSetting m_gs;
      double m_gearRatio;
      QAngularSpeed m_angularspeed_target = 0_rpm;
      BangBangController *m_bangBangController = NULL;
      vex::mutex m_mutex;

      virtual void ConsumeAngularSpeed(double targetVoltage) {
        vex::motor m = vex::motor()
        if(m_angularspeed_target == 0_rpm) {
          m_mg.stop();
        } else {
          double maxRpm = 200;
          if(m_gs == vex::gearSetting::ratio36_1) {
            maxRpm = 100;
          } else if(m_gs == vex::gearSetting::ratio6_1) {
            maxRpm = 600;
          }
          maxRpm = maxRpm * m_gearRatio;
          double rpmVoltage = (m_angularspeed_target.convert(rpm) / maxRpm) * 12000; // 12000 mV limit for V5 motors
          double voltageToApply = targetVoltage * 12000 + rpmVoltage;  // targetVoltage will be either 0 or 1 from the BangBangController. 12000 mV limit
          m_mg.spin(vex::directionType::fwd, voltageToApply, vex::voltageUnits::mV);
        }
      }
  };
} // vpi
