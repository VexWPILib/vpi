// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include "vpi/geometry/VexGpsPose2d.h"
#include "vpi/hal/AbstractSensor.h"
#include "vpi/units/QFrequency.h"
#include "vpi/units/QTime.h"

namespace vpi {
  class VexGpsPose2dGPS : public AbstractSensor {
    public:
      VexGpsPose2dGPS(vex::gps g, int minQuality=90, QTime maxReadingTime=0_ms, QFrequency sampleRate=50_Hz) : 
        m_gps(g),m_minQuality(minQuality), m_maxReadingTime(maxReadingTime), m_sampleRate(sampleRate) {
      }

      /**
       * Returns the reading above the minimum quality threshold.
       * 
       * If maxReadingTime is less than 5_ms, will always return the current reading, regardless of quality
       *
       * If maxReadingTime is more than 5_ms and it takes longer than maxReadingTime, returns a value with 0 quality
       *
       * NOTE: Best if used when robot is stationary
       */
      VexGpsPose2d GetValue() {
        if(m_maxReadingTime < 5_ms) {
          return GetReading();
        }
        int startTime = Brain.Timer.time();
        VexGpsPose2d retval(-99 * meter, -99 * meter, 0 * degree, 0, 0_ms);
        while(Brain.Timer.time() - startTime < m_maxReadingTime.convert(millisecond) && m_gps.quality() < m_minQuality) {
          wait(UnitUtils::convertFrequencyToDelayTime(m_sampleRate).convert(millisecond), msec);
        }
        if(m_gps.quality() < m_minQuality) {
          return retval;
        } else {
          return GetReading();
        }
      }

    protected:
      vex::gps m_gps;
      int m_minQuality;
      QTime m_maxReadingTime;
      QFrequency m_sampleRate;

      VexGpsPose2d GetReading() {
        int q = m_gps.quality();
        double x = m_gps.xPosition(vex::distanceUnits::in);
        double y = m_gps.yPosition(vex::distanceUnits::in);
        double h = m_gps.heading();
        int t = m_gps.timestamp();
        return VexGpsPose2d(x * inch, y * inch, h * degree, q, t * millisecond);
      }
  };
} // end vpi
