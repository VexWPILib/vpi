// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"

namespace vpi {
  class EncoderRotationSensor : public AbstractRotationSensor {
    public:
      EncoderRotationSensor(vex::encoder e) : m_encoder(e) {}

      QAngle GetValue() override {
        m_timestamp = 1000 * (uint32_t)Brain.Timer.value();  // Milliseconds? TODO
        return m_encoder.rotation(vex::rotationUnits::deg) * degree;
      }

      void Reset() override {
        m_encoder.resetRotation();
      }

      QAngularSpeed GetAngularSpeed() override {
        return m_encoder.velocity(vex::velocityUnits::rpm) * rpm;
      }

    protected:
      vex::encoder m_encoder;
  };
} // end vpi
