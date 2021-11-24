// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>
#include <vector>

#undef __ARM_NEON__
#undef __ARM_NEON
#define  EIGEN_MAX_STATIC_ALIGN_BYTES 0
#include "Eigen/Core"

#include "vpi/geometry/Pose2d.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QLength.h"

namespace vpi {
/**
 * Represents a two-dimensional parametric spline that interpolates between two
 * points.
 *
 * @tparam Degree The degree of the spline.
 */
template <int Degree>
class Spline {
 public:
  Spline() = default;

  Spline(const Spline&) = default;
  Spline& operator=(const Spline&) = default;

  Spline(Spline&&) = default;
  Spline& operator=(Spline&&) = default;

  virtual ~Spline() = default;

  /**
   * Represents a control vector for a spline.
   *
   * Each element in each array represents the value of the derivative at the
   * index. For example, the value of x[2] is the second derivative in the x
   * dimension.
   */
  struct ControlVector {
    std::array<double, (Degree + 1) / 2> x;
    std::array<double, (Degree + 1) / 2> y;
  };

  /**
   * Gets the pose and curvature at some point t on the spline.
   *
   * @param t The point t
   * @return The pose and curvature at that point.
   */
  Pose2dWithCurvature GetPoint(double t) const {
    Eigen::Vector<double, Degree + 1> polynomialBases;

    // Populate the polynomial bases
    for (int i = 0; i <= Degree; i++) {
      polynomialBases(i) = std::pow(t, Degree - i);
    }

    // This simply multiplies by the coefficients. We need to divide out t some
    // n number of times where n is the derivative we want to take.
    Eigen::Vector<double, 6> combined = Coefficients() * polynomialBases;

    double dx, dy, ddx, ddy;

    // If t = 0, all other terms in the equation cancel out to zero. We can use
    // the last x^0 term in the equation.
    if (t == 0.0) {
      dx = Coefficients()(2, Degree - 1);
      dy = Coefficients()(3, Degree - 1);
      ddx = Coefficients()(4, Degree - 2);
      ddy = Coefficients()(5, Degree - 2);
    } else {
      // Divide out t for first derivative.
      dx = combined(2) / t;
      dy = combined(3) / t;

      // Divide out t for second derivative.
      ddx = combined(4) / t / t;
      ddy = combined(5) / t / t;
    }

    // Find the curvature.
    const auto curvature =
        (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * std::hypot(dx, dy));

    Pose2d p(FromVector(combined.template block<2, 1>(0, 0)), Rotation2d(dx, dy));
    Pose2dWithCurvature retval(p, curvature * radpermeter);
    return retval;
  }

 protected:
  /**
   * Returns the coefficients of the spline.
   *
   * @return The coefficients of the spline.
   */
  virtual Eigen::Matrix<double, 6, Degree + 1> Coefficients() const = 0;

  /**
   * Converts a Translation2d into a vector that is compatible with Eigen.
   *
   * @param translation The Translation2d to convert.
   * @return The vector.
   */
  static Eigen::Vector2d ToVector(const Translation2d& translation) {
    return Eigen::Vector2d{translation.X().convert(meter), translation.Y().convert(meter)};
  }

  /**
   * Converts an Eigen vector into a Translation2d.
   *
   * @param vector The vector to convert.
   * @return The Translation2d.
   */
  static Translation2d FromVector(const Eigen::Vector2d& vector) {
    return Translation2d(vector(0) * meter, vector(1) * meter);
  }
};
}  // namespace vpi