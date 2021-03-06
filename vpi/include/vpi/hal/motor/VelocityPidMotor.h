// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <thread>

#include "vex.h"
#include "vpi/pid/PIDVelocityController.h"
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
   * PIDFParameters pidf_params = PIDFParameters(12.0/1800.0,0,0,0); // TODO - tune
   * VelocityPidMotor vm(ml, pidf_params, 84.0 / 12.0);
   * vm.SetAngularSpeed(100_rpm);
   * wait(5, vex::seconds);
   * vm.SetAngularSpeed(200_rpm);
   * wait(5, vex::seconds);
   * @endcode
   * 
   */
  class VelocityPidMotor {
    public:
      VelocityPidMotor(vex::motor m, PIDFParameters pidf, double gearRatio = 1.0) 
          : m_mg({m}), m_pidf(pidf)
      {
        m_gearRatio = gearRatio;
        m_velocityPidController = new PIDVelocityController(pidf, [this] {return this->GetCurrentAngularSpeed().convert(rpm);},
                                              [this](double v) {this->ConsumeAngularSpeed(v);});
      }

      VelocityPidMotor(vex::motor_group mg, PIDFParameters pidf, double gearRatio = 1.0) 
          : m_mg(mg), m_pidf(pidf)
      {
        m_gearRatio = gearRatio;
      }

      virtual ~VelocityPidMotor() {
        Disable();
        if(m_velocityPidController != NULL) {
          m_velocityPidController->Disable();
          m_velocityPidController = NULL;
        }
      }

      void SetAngularSpeed(QAngularSpeed s) {
        m_mutex.lock();
        m_angularspeed_target = s;
        m_mutex.unlock();

        if(m_velocityPidController != NULL) {
          m_velocityPidController->SetSetpoint(m_angularspeed_target.convert(rpm));
        }        
      }

      void Enable() {
        if(m_velocityPidController != NULL) {
          m_velocityPidController->Enable();
        }
      }
      
      void Disable() {
        if(m_velocityPidController != NULL) {
          m_velocityPidController->Disable();
        }        
      }

      bool IsEnabled() {
        if(m_velocityPidController != NULL) {
          return m_velocityPidController->CheckEnabled();
        }
        return false;
      }

      virtual QAngularSpeed GetCurrentAngularSpeed() {
        m_mutex.lock();
        QAngularSpeed curRpm = m_mg.velocity(vex::velocityUnits::rpm) * m_gearRatio * rpm;
        m_mutex.unlock();
        return curRpm;
      }

    protected:
      vex::motor_group m_mg;
      PIDFParameters m_pidf;
      double m_gearRatio;
      QAngularSpeed m_angularspeed_target = 0_rpm;
      PIDVelocityController *m_velocityPidController = NULL;
      vex::mutex m_mutex;

      virtual void ConsumeAngularSpeed(double targetVoltage) {
        m_mg.spin(vex::directionType::fwd, targetVoltage, vex::voltageUnits::volt);
      }
  };
} // vpi
