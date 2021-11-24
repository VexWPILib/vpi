// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * MIT License
 *
 * Copyright (c) 2018 Team 254
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <stack>
#include <string>
#include <utility>
#include <vector>

#include <stdio.h>

#include "vpi/spline/Spline.h"
#include "vpi/units/QAngle.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QLength.h"

namespace vpi {

/**
 * Class used to parameterize a spline by its arc length.
 */
class SplineParameterizer {
 public:
  /**
   * Parameterizes the spline. This method breaks up the spline into various
   * arcs until their dx, dy, and dtheta are within specific tolerances.
   *
   * @param spline The spline to parameterize.
   * @param t0 Starting internal spline parameter. It is recommended to leave
   * this as default.
   * @param t1 Ending internal spline parameter. It is recommended to leave this
   * as default.
   *
   * @return A vector of poses and curvatures that represents various points on
   * the spline.
   */
  template <int Dim>
  static std::vector<Pose2dWithCurvature> Parameterize(const Spline<Dim>& spline,
                                                     double t0 = 0.0,
                                                     double t1 = 1.0) {
    std::vector<Pose2dWithCurvature> splinePoints;

    // The parameterization does not add the initial point. Let's add that.
    splinePoints.push_back(spline.GetPoint(t0));

    // We use an "explicit stack" to simulate recursion, instead of a recursive
    // function call This give us greater control, instead of a stack overflow
    std::stack<StackContents> stack;
    stack.emplace(StackContents{t0, t1});

    StackContents current;
    Pose2dWithCurvature start;
    Pose2dWithCurvature end;
    int iterations = 0;

    while (!stack.empty()) {
      current = stack.top();
      stack.pop();
      start = spline.GetPoint(current.t0);
      end = spline.GetPoint(current.t1);

      const auto twist = start.pose.Log(end.pose);

      if (fabs(twist.dy.convert(meter)) > kMaxDy.convert(meter) ||
          fabs(twist.dx.convert(meter)) > kMaxDx.convert(meter) ||
          fabs(twist.dtheta.convert(radian)) > kMaxDtheta.convert(radian)) {
        stack.emplace(StackContents{(current.t0 + current.t1) / 2, current.t1});
        stack.emplace(StackContents{current.t0, (current.t0 + current.t1) / 2});
      } else {
        splinePoints.push_back(spline.GetPoint(current.t1));
      }

      if (iterations++ >= kMaxIterations) {
        printf("EXCEPTION!!!!!! Could not parameterize a malformed spline. "
            "This means that you probably had two or more adjacent "
            "waypoints that were very close together with headings "
            "in opposing directions.\n");
        /*
        throw MalformedSplineException(
            "Could not parameterize a malformed spline. "
            "This means that you probably had two or more adjacent "
            "waypoints that were very close together with headings "
            "in opposing directions.");
        */
        // No exceptions allowed in VexCode Pro...
        break;
      }
    }

    return splinePoints;
  }

 private:
  // Constraints for spline parameterization.
  static constexpr QLength kMaxDx = 1_in;
  static constexpr QLength kMaxDy = 0.5_in;
  static constexpr QAngle kMaxDtheta = 0.0872_rad;

  struct StackContents {
    double t0;
    double t1;
  };

  /**
   * A malformed spline does not actually explode the LIFO stack size. Instead,
   * the stack size stays at a relatively small number (e.g. 30) and never
   * decreases. Because of this, we must count iterations. Even long, complex
   * paths don't usually go over 300 iterations, so hitting this maximum should
   * definitely indicate something has gone wrong.
   */
  static constexpr int kMaxIterations = 5000;

  friend class CubicHermiteSplineTest;
  friend class QuinticHermiteSplineTest;
};
}  // namespace vpi