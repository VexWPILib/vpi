// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"

namespace vpi {
  class MotorGroupRotationSensor : public AbstractRotationSensor {
    public:
      MotorGroupRotationSensor(vex::motor_group mg, vex::gearSetting gs) 
          : m_motorgroup(mg), m_gearSetting(gs)
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
        m_timestamp = 1000 * (uint32_t)Brain.Timer.value();  // Milliseconds? TODO
        return m_motorgroup.rotation(vex::rotationUnits::deg) * degree;
      }

      void Reset() override {
        m_motorgroup.resetRotation();
      }

      QAngularSpeed GetAngularSpeed() override {
        // TODO - Not sure if this should be multiplied by gearMultiplier
        return m_motorgroup.velocity(vex::velocityUnits::rpm) * rpm;
      }

    protected:
      vex::motor_group m_motorgroup;
      vex::gearSetting m_gearSetting;
      double m_gearMultiplier = 1.0;
  };
} // end vpi
