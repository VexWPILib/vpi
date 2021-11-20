// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace vpi {

class VpiUtils {
  public:
  static double ApplyDeadband(double value, double deadband) {
    if (std::abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  template <typename T>
  static T clip(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
  }

  /**
  * Returns modulus of input.
  *
  * @param input        Input value to wrap.
  * @param minimumInput The minimum value expected from the input.
  * @param maximumInput The maximum value expected from the input.
  */
  template <typename T>
  static T InputModulus(T input, T minimumInput, T maximumInput) {
    T modulus = maximumInput - minimumInput;

    // Wrap input if it's above the maximum input
    int numMax = (input - minimumInput) / modulus;
    input -= numMax * modulus;

    // Wrap input if it's below the minimum input
    int numMin = (input - maximumInput) / modulus;
    input -= numMin * modulus;

    return input;
  }

  /**
    * Linearly interpolates between two values.
    *
    * @param startValue The start value.
    * @param endValue The end value.
    * @param t The fraction for interpolation.
    *
    * @return The interpolated value.
    */
  template <typename T>
  static T Lerp(const T& startValue, const T& endValue, const double t) {
    return startValue + (endValue - startValue) * t;
  }

}; // class VpiUtils

/**
* Returns the sign value of the given value.
*
* @return 1 if the value is positive, -1 if the value is negative, and 0 if
*         the value is 0.
*/
template <class T> inline int sgn(T v) {
  return (v > T(0)) - (v < T(0));
}

inline bool
NearlyEqual(const double& a, const double& b, double epsilon = 1e-5) {
  return std::fabs(a - b) < epsilon;
}

}  // namespace vpi

// From: https://stackoverflow.com/questions/17902405/how-to-implement-make-unique-function-in-c11
namespace std {
    template<class T> struct _Unique_if {
        typedef unique_ptr<T> _Single_object;
    };

    template<class T> struct _Unique_if<T[]> {
        typedef unique_ptr<T[]> _Unknown_bound;
    };

    template<class T, size_t N> struct _Unique_if<T[N]> {
        typedef void _Known_bound;
    };

    template<class T, class... Args>
        typename _Unique_if<T>::_Single_object
        make_unique(Args&&... args) {
            return unique_ptr<T>(new T(std::forward<Args>(args)...));
        }

    template<class T>
        typename _Unique_if<T>::_Unknown_bound
        make_unique(size_t n) {
            typedef typename remove_extent<T>::type U;
            return unique_ptr<T>(new U[n]());
        }

    template<class T, class... Args>
        typename _Unique_if<T>::_Known_bound
        make_unique(Args&&...) = delete;
}
