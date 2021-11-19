// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/units/QAngle.h"
#include "vpi/units/QAngularSpeed.h"

namespace vpi {
/**
 * An abstraction layer for the various sensors that provide rotation information
 * such as:
 * * 3-wire Encoder (the big Red Sensor)
 * * Motors
 * * MotorGroups
 * * 3-wire potentiometer
 * * V5 Rotation Sensor
 */
  class AbstractRotationSensor {
    public:
      /**
      * Virtual method for getting the sensor's rotation value
      */
      virtual QAngle GetValue() = 0;

      /**
      * Virtual method for resetting the sensor to zero
      */
      virtual void Reset() = 0;

      /**
      * Virtual method for getting the sensor's rotation value
      */
      virtual QAngularSpeed GetAngularSpeed() = 0;

      QTime GetReadingTimestamp() 
      {
        return m_timestamp * millisecond;
      }

    protected:
      uint32_t m_timestamp;
  };
} // end vpi
