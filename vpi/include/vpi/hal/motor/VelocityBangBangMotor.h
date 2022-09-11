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
      VelocityBangBangMotor(vex::motor m, vex::gearSetting gs, double pctBangBang, double tolRpm, double gearRatio = 1.0, QTime period = 10_ms) 
          : m_mg({m}), m_gs(gs)
      {
        m_gearRatio = gearRatio;
        m_pctBB = pctBangBang;
        if(m_pctBB < 0.05 || m_pctBB > 1.0) {
          m_pctBB = .1;
        }

        m_bangBangController = new BangBangController([this] {return this->GetCurrentAngularSpeed().convert(rpm);},
                                              [this](double v) {this->ConsumeAngularSpeed(v);},
                                              period, tolRpm);
      }

      VelocityBangBangMotor(vex::motor_group mg, vex::gearSetting gs, double pctBangBang, double tolRpm, double gearRatio = 1.0, QTime period = 10_ms) 
          : m_mg(mg), m_gs(gs)
      {
        m_gearRatio = gearRatio;
        m_pctBB = pctBangBang;
        if(m_pctBB < 0.05 || m_pctBB > 1.0) {
          m_pctBB = .1;
        }
        m_bangBangController = new BangBangController([this] {return this->GetCurrentAngularSpeed().convert(rpm);},
                                              [this](double v) {this->ConsumeAngularSpeed(v);},
                                              period, tolRpm);
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

      virtual QAngularSpeed GetCurrentAngularSpeed() {
        m_mutex.lock();
        double motorRpm = m_mg.velocity(vex::velocityUnits::rpm);
        QAngularSpeed curRpm = motorRpm * m_gearRatio * rpm;
        m_mutex.unlock();
        return curRpm;
      }

      void SetDebug(bool b) {
        m_mutex.lock();
        m_debug = b;
        m_mutex.unlock();
      }
      
    protected:
      vex::motor_group m_mg;
      vex::gearSetting m_gs;
      double m_gearRatio;
      QAngularSpeed m_angularspeed_target = 0_rpm;
      BangBangController *m_bangBangController = NULL;
      vex::mutex m_mutex;
      bool m_debug = false;
      double m_pctBB = .1;

      virtual void ConsumeAngularSpeed(double targetVoltage) {
        if(m_angularspeed_target == 0_rpm) {
          m_mg.stop(vex::brakeType::coast);
        } else {
          double maxRpm = 200;
          if(m_gs == vex::gearSetting::ratio36_1) {
            maxRpm = 100;
          } else if(m_gs == vex::gearSetting::ratio6_1) {
            maxRpm = 600;
          }
          maxRpm = maxRpm * m_gearRatio;
          double rpmVoltage = (1.05 - m_pctBB) * (m_angularspeed_target.convert(rpm) / maxRpm) * 12000; // 12000 mV limit for V5 motors. Take 90%
          double voltageToApply = rpmVoltage + m_pctBB * targetVoltage * 12000;

          if(m_debug) {
            int curTime = (int)UnitUtils::now().convert(millisecond);
            double mRpm = m_mg.velocity(vex::velocityUnits::rpm);
            double mVolts = m_mg.voltage(vex::voltageUnits::mV);
            double mCurrent = m_mg.current(vex::currentUnits::amp);
            double mPower = m_mg.power(vex::powerUnits::watt);
            double mTemp = m_mg.temperature(vex::temperatureUnits::fahrenheit);
            double mTorque = m_mg.torque(vex::torqueUnits::Nm);
            printf("%d,%d,%d,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n",curTime, 
                                                                (int)(mRpm * m_gearRatio),
                                                                (int)m_angularspeed_target.convert(rpm),
                                                                targetVoltage,
                                                                voltageToApply,
                                                                mVolts,
                                                                mCurrent,
                                                                mPower,
                                                                mTemp,
                                                                mTorque);
          }
          m_mg.spin(vex::directionType::fwd, voltageToApply, vex::voltageUnits::mV);
        }
      }
  };
} // vpi
