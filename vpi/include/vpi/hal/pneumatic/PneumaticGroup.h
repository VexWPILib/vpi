// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include "vex.h"
#include <vector>

namespace vpi {
  
  class PneumaticGroup  {

    private:
      std::vector<vex::pneumatics> m_pneumatics;

      void _addPneumatic( vex::pneumatics &p );
      
      template <typename... Args>
      void _addPneumatic(  vex::pneumatics &p1, Args &... p2 ) {
         _addPneumatic( p1 );
         _addPneumatic( p2... );
      }

    public:      
      PneumaticGroup();
      ~PneumaticGroup();
      
      template <typename... Args>
      PneumaticGroup( vex::pneumatics &p1, Args &... p2 ) : PneumaticGroup() {
        _addPneumatic( p1 );
        _addPneumatic( p2... );
      }

      template <typename... Args>
      void operator()( vex::pneumatics &p1, Args &... p2 ) {
        _addPneumatic( p1 );
        _addPneumatic( p2... );
      }
     
      /** 
       * @brief return the number of pneumatics in the pneumatics group
       * @return number of pneumatics
       */
      int32_t count(void) {return m_pneumatics.size();}
     
      /**
       * @brief Sets the pneumatics device to the solenoid open state allowing air to flow into the cylinder.
       */
      void open() {
        for (auto & p : m_pneumatics) {
            p.open();
        }
      }

      /**
       * @brief Sets the pneumatics device to the solenoid close state stopping air flowing into the cylinder.
       */
      void close() {
        for (auto & p : m_pneumatics) {
            p.close();
        }
      }
  };
};