// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <thread>

#include "vex.h"
#include "vpi/ui/Graph.h"
#include "vpi/units/QTime.h"

namespace vpi {
  /**
   * Example usage:
   *
   * @code{.cpp}
   * vex::motor m = motor(PORT1, ratio18_1, false);  // also works for vex::motor_group
   * GraphedMotor vm(ml, top_left_x, top_left_y, width, height);
   * vm.SetAngularSpeed(100_rpm);
   * wait(5, vex::seconds);
   * vm.SetAngularSpeed(200_rpm);
   * wait(5, vex::seconds);
   * @endcode
   * 
   */
  class GraphedMotor {
    public:
        GraphedMotor(vex::brain::lcd &screen, vex::motor m, int top_left_x, int top_left_y, int width, int height, QTime period = 20_ms) 
        : m_motor(m), m_graph(screen, 4, top_left_x, top_left_y, width, height) {
            if (period < 5 * millisecond) {
                m_period = 5_ms;
                logger.log(Logger::LogLevel::WARN, "Controller period defaulted to 20ms");
            }
            m_graph.SetColor(0, vex::color::white); // RPM in percent
            m_graph.SetMaxY(0, 100);
            m_graph.SetColor(1, vex::color::blue);  // Volts in percent
            m_graph.SetMaxY(1, 100);
            m_graph.SetColor(2, vex::color::green); // Current in percent
            m_graph.SetMaxY(2, 100);
            m_graph.SetColor(3, vex::color::red);   // Temp
            m_graph.SetMaxY(3, 100);
        }

        virtual ~GraphedMotor() {
            Disable();
        }

        void Enable() {
            m_mutex.lock();
            m_enabled = true;
            m_mutex.unlock();
            // Task management
            if(m_controlTask != NULL) {
                m_controlTask->stop();
            }
            m_controlTask = new vex::task(GraphedMotor::_trampoline, static_cast<void *>(this));
        }

        void Disable() {
            // Task management
            if(m_controlTask != NULL) {
                m_controlTask->stop();
            }
            m_mutex.lock();
            m_enabled = false;
            m_mutex.unlock();
        }

        bool IsEnabled() {
            m_mutex.lock();
            bool retval = m_enabled;
            m_mutex.unlock();
            return retval;
        }
     
    protected:
        vex::motor m_motor;
        // The period (in seconds) of the control loop running this controller
        QTime m_period;
        vex::mutex m_mutex;
        bool m_enabled = false;
        Graph m_graph;

    private:
        vex::task *m_controlTask = NULL;

        void ControlLoop() {
            while(1) {
                if(IsEnabled()) {
                    int mRpm = (int)m_motor.velocity(vex::percentUnits::pct);
                    int mVolts = (int)(m_motor.voltage(vex::voltageUnits::mV) / 12000);
                    int mCurrent = (int)m_motor.current(vex::percentUnits::pct);
                    int mTemp = (int)m_motor.temperature(vex::temperatureUnits::celsius);
                    m_graph.AddPoint(0, mRpm);
                    m_graph.AddPoint(1, mVolts);
                    m_graph.AddPoint(2, mCurrent);
                    m_graph.AddPoint(3, mTemp);
                }
                this_thread::sleep_for(m_period.convert(millisecond));
            }
        }

        static int _trampoline(void *p_this) {
            GraphedMotor *p = (GraphedMotor *)p_this;
            while(1) {
                p->ControlLoop();
                // Sleep is handled in the ControlLoop function
            }

            return 0;
        }
  };
} // vpi
