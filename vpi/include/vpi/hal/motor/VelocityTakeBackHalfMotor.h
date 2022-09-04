// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <thread>

#include "vex.h"
#include "vpi/log/Logger.h"
#include "vpi/controller/TakeBackHalfController.h"
#include "vpi/units/QAngularSpeed.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {
  /**
   * Example usage:
   *
   * @code{.cpp}
   * vex::motor m = motor(PORT1, ratio18_1, false);  // also works for vex::motor_group
   * // Should go from RPM to Volts for scaling, so p might be 12 (V) / 200 (rpm)
   * // for an ungeared green-cartridge motor
   * VelocityTakeBackHalfMotor vm(ml, 84.0 / 12.0);
   * vm.SetAngularSpeed(100_rpm);
   * wait(5, vex::seconds);
   * vm.SetAngularSpeed(200_rpm);
   * wait(5, vex::seconds);
   * @endcode
   * 
   */
  class VelocityTakeBackHalfMotor {
    public:
      VelocityTakeBackHalfMotor(vex::motor m, double g, double gearRatio = 1.0, QTime period = 10_ms) 
          : m_mg({m})
      {
        m_gearRatio = gearRatio;
        m_tbhController = new TakeBackHalfController(g, [this] {return this->GetCurrentAngularSpeed().convert(rpm);},
                                              [this](double v) {this->ConsumeAngularSpeed(v);},
                                              period);
      }

      VelocityTakeBackHalfMotor(vex::motor_group mg, double g, double gearRatio = 1.0, QTime period = 10_ms) 
          : m_mg(mg)
      {
        m_gearRatio = gearRatio;
        m_tbhController = new TakeBackHalfController(g, [this] {return this->GetCurrentAngularSpeed().convert(rpm);},
                                              [this](double v) {this->ConsumeAngularSpeed(v);},
                                              period);
      }

      virtual ~VelocityTakeBackHalfMotor() {
        Disable();
        if(m_tbhController != NULL) {
          m_tbhController->Disable();
          m_tbhController = NULL;
        }
      }

      void SetAngularSpeed(QAngularSpeed s) {
        m_mutex.lock();
        m_angularspeed_target = s;
        m_mutex.unlock();

        if(m_tbhController != NULL) {
          m_tbhController->SetSetpoint(m_angularspeed_target.convert(rpm));
        }
      }

      void Enable() {
        if(m_tbhController != NULL) {
          m_tbhController->Enable();
        }
      }
      
      void Disable() {
        if(m_tbhController != NULL) {
          m_tbhController->Disable();
        }        
      }

      bool IsEnabled() {
        if(m_tbhController != NULL) {
          return m_tbhController->CheckEnabled();
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
      TakeBackHalfController *m_tbhController = NULL;
      vex::mutex m_mutex;
      bool m_debug = false;

      virtual void ConsumeAngularSpeed(double targetVoltage) {
        if(m_angularspeed_target == 0_rpm) {
          m_mg.stop(vex::brakeType::coast);
        } else {
          if(m_debug) {
            int curTime = (int)UnitUtils::now().convert(millisecond);
            double mRpm = m_mg.velocity(vex::velocityUnits::rpm);
            double mVolts = m_mg.voltage(vex::voltageUnits::mV);
            double mCurrent = m_mg.current(vex::currentUnits::amp);
            double mPower = m_mg.power(vex::powerUnits::watt);
            double mTemp = m_mg.temperature(vex::temperatureUnits::fahrenheit);
            double mTorque = m_mg.torque(vex::torqueUnits::Nm);
            printf("%d,%d,%d,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f\n",curTime, 
                                                                (int)(mRpm * m_gearRatio),
                                                                (int)m_angularspeed_target.convert(rpm),
                                                                targetVoltage,
                                                                mVolts,
                                                                mCurrent,
                                                                mPower,
                                                                mTemp,
                                                                mTorque);
          }
          m_mg.spin(vex::directionType::fwd, targetVoltage, vex::voltageUnits::mV);
        }
      }
  };
} // vpi
