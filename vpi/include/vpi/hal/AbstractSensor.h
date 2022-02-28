// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vpi/units/QTime.h"

namespace vpi {
  class AbstractSensor {
    public:
      QTime GetReadingTimestamp() 
      {
        return m_timestamp * millisecond;
      }

    protected:
      uint32_t m_timestamp;
  };
} // end vpi
