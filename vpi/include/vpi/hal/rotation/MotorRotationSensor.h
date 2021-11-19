// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"

namespace vpi {
  class MotorRotationSensor : public AbstractRotationSensor {
    public:
      MotorRotationSensor(vex::motor m, vex::gearSetting gs) 
        : m_motor(m), m_gearSetting(gs)
      {
        /*
          if(gs == vex::ratio36_1) {
            m_gearMultiplier = 3.0;
          } else if(gs == vex::ratio6_1) {
            m_gearMultiplier = 0.5;
          }
        */
      }

      QAngle GetValue() override {
        m_timestamp = m_motor.timestamp();  // Milliseconds? TODO
        
        return m_motor.rotation(vex::rotationUnits::deg) * m_gearMultiplier * degree;
      }

      void Reset() override {
        m_motor.resetRotation();
      }

      QAngularSpeed GetAngularSpeed() override {
        return m_motor.velocity(vex::velocityUnits::rpm) * m_gearMultiplier * rpm;
      }

    protected:
      vex::motor m_motor;
      vex::gearSetting m_gearSetting;
      double m_gearMultiplier = 1.0;
  };
} // end vpi
