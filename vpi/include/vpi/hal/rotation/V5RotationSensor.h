// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"

namespace vpi {
  class V5RotationSensor : public AbstractRotationSensor {
    public:
      V5RotationSensor(vex::rotation r) : m_r(r) {}

      QAngle GetValue() override {
        m_timestamp = m_r.timestamp(); // Milliseconds? TODO
        return (m_r.angle(vex::rotationUnits::deg) * degree);
      }

      void Reset() override {
        m_r.resetPosition();
      }

      QAngularSpeed GetAngularSpeed() override {
        return m_r.velocity(vex::velocityUnits::rpm) * rpm;
      }

    protected:
      vex::rotation m_r;
  };
} // end vpi
