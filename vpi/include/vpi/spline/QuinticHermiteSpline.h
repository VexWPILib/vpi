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
 * Represents a hermite spline of degree 5.
 */
class QuinticHermiteSpline : public Spline<5> {
 public:
  /**
   * Constructs a quintic hermite spline with the specified control vectors.
   * Each control vector contains into about the location of the point, its
   * first derivative, and its second derivative.
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
  QuinticHermiteSpline(std::array<double, 3> xInitialControlVector,
                       std::array<double, 3> xFinalControlVector,
                       std::array<double, 3> yInitialControlVector,
                       std::array<double, 3> yFinalControlVector);

 protected:
  /**
   * Returns the coefficients matrix.
   * @return The coefficients matrix.
   */
  Eigen::Matrix<double, 6, 6> Coefficients() const override {
    return m_coefficients;
  }

 private:
  Eigen::Matrix<double, 6, 6> m_coefficients =
      Eigen::Matrix<double, 6, 6>::Zero();

  /**
   * Returns the hermite basis matrix for quintic hermite spline interpolation.
   * @return The hermite basis matrix for quintic hermite spline interpolation.
   */
  static Eigen::Matrix<double, 6, 6> MakeHermiteBasis() {
    // Given P(i), P'(i), P''(i), P(i+1), P'(i+1), P''(i+1), the control
    // vectors, we want to find the coefficients of the spline
    // P(t) = a5 * t^5 + a4 * t^4 + a3 * t^3 + a2 * t^2 + a1 * t + a0.
    //
    // P(i)     = P(0)   = a0
    // P'(i)    = P'(0)  = a1
    // P''(i)   = P''(0) = 2 * a2
    // P(i+1)   = P(1)   = a5 + a4 + a3 + a2 + a1 + a0
    // P'(i+1)  = P'(1)  = 5 * a5 + 4 * a4 + 3 * a3 + 2 * a2 + a1
    // P''(i+1) = P''(1) = 20 * a5 + 12 * a4 + 6 * a3 + 2 * a2
    //
    // [ P(i)     ] = [  0  0  0  0  0  1 ][ a5 ]
    // [ P'(i)    ] = [  0  0  0  0  1  0 ][ a4 ]
    // [ P''(i)   ] = [  0  0  0  2  0  0 ][ a3 ]
    // [ P(i+1)   ] = [  1  1  1  1  1  1 ][ a2 ]
    // [ P'(i+1)  ] = [  5  4  3  2  1  0 ][ a1 ]
    // [ P''(i+1) ] = [ 20 12  6  2  0  0 ][ a0 ]
    //
    // To solve for the coefficients, we can invert the 6x6 matrix and move it
    // to the other side of the equation.
    //
    // [ a5 ] = [  -6.0  -3.0  -0.5   6.0  -3.0   0.5 ][ P(i)     ]
    // [ a4 ] = [  15.0   8.0   1.5 -15.0   7.0  -1.0 ][ P'(i)    ]
    // [ a3 ] = [ -10.0  -6.0  -1.5  10.0  -4.0   0.5 ][ P''(i)   ]
    // [ a2 ] = [   0.0   0.0   0.5   0.0   0.0   0.0 ][ P(i+1)   ]
    // [ a1 ] = [   0.0   1.0   0.0   0.0   0.0   0.0 ][ P'(i+1)  ]
    // [ a0 ] = [   1.0   0.0   0.0   0.0   0.0   0.0 ][ P''(i+1) ]

    static const Eigen::Matrix<double, 6, 6> basis{
        {-06.0, -03.0, -00.5, +06.0, -03.0, +00.5},
        {+15.0, +08.0, +01.5, -15.0, +07.0, -01.0},
        {-10.0, -06.0, -01.5, +10.0, -04.0, +00.5},
        {+00.0, +00.0, +00.5, +00.0, +00.0, +00.0},
        {+00.0, +01.0, +00.0, +00.0, +00.0, +00.0},
        {+01.0, +00.0, +00.0, +00.0, +00.0, +00.0}};
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
  static Eigen::Vector<double, 6> ControlVectorFromArrays(
      std::array<double, 3> initialVector, std::array<double, 3> finalVector) {
    return Eigen::Vector<double, 6>{initialVector[0], initialVector[1],
                                    initialVector[2], finalVector[0],
                                    finalVector[1],   finalVector[2]};
  }
};
}  // namespace vpi