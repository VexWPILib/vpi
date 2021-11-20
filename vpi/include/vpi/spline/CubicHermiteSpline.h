// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#undef __ARM_NEON__
#undef __ARM_NEON
#define  EIGEN_MAX_STATIC_ALIGN_BYTES 0
#include "Eigen/Core"

#include "vpi/spline/Spline.h"

namespace vpi {
/**
 * Represents a hermite spline of degree 3.
 */
class CubicHermiteSpline : public Spline<3> {
 public:
  /**
   * Constructs a cubic hermite spline with the specified control vectors. Each
   * control vector contains info about the location of the point and its first
   * derivative.
   *
   * @param xInitialControlVector The control vector for the initial point in
   * the x dimension.
   * @param xFinalControlVector The control vector for the final point in
   * the x dimension.
   * @param yInitialControlVector The control vector for the initial point in
   * the y dimension.
   * @param yFinalControlVector The control vector for the final point in
   * the y dimension.
   */
  CubicHermiteSpline(std::array<double, 2> xInitialControlVector,
                     std::array<double, 2> xFinalControlVector,
                     std::array<double, 2> yInitialControlVector,
                     std::array<double, 2> yFinalControlVector);

 protected:
  /**
   * Returns the coefficients matrix.
   * @return The coefficients matrix.
   */
  Eigen::Matrix<double, 6, 3 + 1> Coefficients() const override {
    return m_coefficients;
  }

 private:
  Eigen::Matrix<double, 6, 4> m_coefficients =
      Eigen::Matrix<double, 6, 4>::Zero();

  /**
   * Returns the hermite basis matrix for cubic hermite spline interpolation.
   * @return The hermite basis matrix for cubic hermite spline interpolation.
   */
  static Eigen::Matrix<double, 4, 4> MakeHermiteBasis() {
    // Given P(i), P'(i), P(i+1), P'(i+1), the control vectors, we want to find
    // the coefficients of the spline P(t) = a3 * t^3 + a2 * t^2 + a1 * t + a0.
    //
    // P(i)    = P(0)  = a0
    // P'(i)   = P'(0) = a1
    // P(i+1)  = P(1)  = a3 + a2 + a1 + a0
    // P'(i+1) = P'(1) = 3 * a3 + 2 * a2 + a1
    //
    // [ P(i)    ] = [ 0 0 0 1 ][ a3 ]
    // [ P'(i)   ] = [ 0 0 1 0 ][ a2 ]
    // [ P(i+1)  ] = [ 1 1 1 1 ][ a1 ]
    // [ P'(i+1) ] = [ 3 2 1 0 ][ a0 ]
    //
    // To solve for the coefficients, we can invert the 4x4 matrix and move it
    // to the other side of the equation.
    //
    // [ a3 ] = [  2  1 -2  1 ][ P(i)    ]
    // [ a2 ] = [ -3 -2  3 -1 ][ P'(i)   ]
    // [ a1 ] = [  0  1  0  0 ][ P(i+1)  ]
    // [ a0 ] = [  1  0  0  0 ][ P'(i+1) ]

    static const Eigen::Matrix<double, 4, 4> basis{{+2.0, +1.0, -2.0, +1.0},
                                                   {-3.0, -2.0, +3.0, -1.0},
                                                   {+0.0, +1.0, +0.0, +0.0},
                                                   {+1.0, +0.0, +0.0, +0.0}};
    return basis;
  }

  /**
   * Returns the control vector for each dimension as a matrix from the
   * user-provided arrays in the constructor.
   *
   * @param initialVector The control vector for the initial point.
   * @param finalVector The control vector for the final point.
   *
   * @return The control vector matrix for a dimension.
   */
  static Eigen::Vector4d ControlVectorFromArrays(
      std::array<double, 2> initialVector, std::array<double, 2> finalVector) {
    return Eigen::Vector4d{initialVector[0], initialVector[1], finalVector[0],
                           finalVector[1]};
  }
};
}  // namespace vpi