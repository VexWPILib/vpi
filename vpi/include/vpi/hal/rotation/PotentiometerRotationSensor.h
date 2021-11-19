// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"

namespace vpi {
  class PotentiometerRotationSensor : public AbstractRotationSensor {
    public:
      PotentiometerRotationSensor(vex::pot p) : m_pot(p) {}

      QAngle GetValue() override {
        m_timestamp = 1000 * (uint32_t)Brain.Timer.value();  // Milliseconds? TODO
        return (m_pot.angle(vex::rotationUnits::deg) * degree - m_lastReset);
      }

      void Reset() override {
        m_lastReset = m_pot.angle(vex::rotationUnits::deg) * degree;
      }

      QAngularSpeed GetAngularSpeed() override {
        return 0 * rpm;
      }

    protected:
      vex::pot m_pot;
      QAngle m_lastReset = 0_rad;
  };
} // end vpi
