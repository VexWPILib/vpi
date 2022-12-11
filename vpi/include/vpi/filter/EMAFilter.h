// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once
namespace vpi {

  class EMAFilter {
    private:
      double m_kA;
      double m_value;

    public:
      /**
       * @param kA
       *        How much the filter will condider previous information. A value of 1
       *        will make the input the full value of the output, a value of 0 will
       *        make the input have no effect on the output.
       */
      EMAFilter(double kA) { m_kA = kA; m_value = 0;}

      /**
       * @param newInput
       *        The raw input value to filter
       *
       * @return Filter output
       */
      double filter(double newInput) {
        m_value = newInput * m_kA + m_value * (1 - m_kA);
        return m_value;
      }

      /**
       * @param newInput
       *        The raw input value to filter
       * @param kA
       *        How much the filter will condider previous information. A value of 1
       *        will make the input the full value of the output, a value of 0 will
       *        make the input have no effect on the output.
       *
       * @return Filter output
       */
      double filter(double newInput, double kA) {
        m_kA = kA;
        return filter(newInput);
      }

      double getkA() const { return m_kA; }

      double getCurrentValue() const {return m_value; }      
  };
}