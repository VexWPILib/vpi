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

#include "vpi/trajectory/TrajectoryParameterizer.h"

using namespace vpi;

Trajectory TrajectoryParameterizer::TimeParameterizeTrajectory(
    const std::vector<PoseWithCurvature>& points,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    QSpeed startVelocity,
    QSpeed endVelocity,
    QSpeed maxVelocity,
    QAcceleration maxAcceleration, bool reversed) {
  std::vector<ConstrainedState> constrainedStates(points.size());

  ConstrainedState predecessor;
  predecessor.pose = points.front();
  predecessor.distance = 0_m;
  predecessor.maxVelocity = startVelocity;
  predecessor.minAcceleration = -1.0 * maxAcceleration;
  predecessor.minAcceleration = maxAcceleration;

  constrainedStates[0] = predecessor;

  // Forward pass
  for (unsigned int i = 0; i < points.size(); i++) {
    auto& constrainedState = constrainedStates[i];
    constrainedState.pose = points[i];

    // Begin constraining based on predecessor
    QLength ds = constrainedState.pose.first.Translation().Distance(
        predecessor.pose.first.Translation());
    constrainedState.distance = ds + predecessor.distance;

    // We may need to iterate to find the maximum end velocity and common
    // acceleration, since acceleration limits may be a function of velocity.
    while (true) {
      // Enforce global max velocity and max reachable velocity by global
      // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).

      constrainedState.maxVelocity = mps * fmin(
          maxVelocity.convert(mps),
          sqrt(predecessor.maxVelocity * predecessor.maxVelocity +
              predecessor.maxAcceleration * ds * 2.0).convert(mps));

      constrainedState.minAcceleration = -1.0 * maxAcceleration;
      constrainedState.maxAcceleration = maxAcceleration;

      // At this point, the constrained state is fully constructed apart from
      // all the custom-defined user constraints.
      for (const auto& constraint : constraints) {
        constrainedState.maxVelocity = mps * fmin(
            constrainedState.maxVelocity.convert(mps),
            constraint->MaxVelocity(constrainedState.pose.first,
                                    constrainedState.pose.second,
                                    constrainedState.maxVelocity).convert(mps));
      }

      // Now enforce all acceleration limits.
      EnforceAccelerationLimits(reversed, constraints, &constrainedState);

      if (ds.convert(meter) < kEpsilon) {
        break;
      }

      // If the actual acceleration for this state is higher than the max
      // acceleration that we applied, then we need to reduce the max
      // acceleration of the predecessor and try again.
      QAcceleration actualAcceleration =
          (constrainedState.maxVelocity * constrainedState.maxVelocity -
           predecessor.maxVelocity * predecessor.maxVelocity) /
          (ds * 2.0);

      // If we violate the max acceleration constraint, let's modify the
      // predecessor.
      if (constrainedState.maxAcceleration < actualAcceleration - 1E-6_mps2) {
        predecessor.maxAcceleration = constrainedState.maxAcceleration;
      } else {
        // Constrain the predecessor's max acceleration to the current
        // acceleration.
        if (actualAcceleration > predecessor.minAcceleration + 1E-6_mps2) {
          predecessor.maxAcceleration = actualAcceleration;
        }
        // If the actual acceleration is less than the predecessor's min
        // acceleration, it will be repaired in the backward pass.
        break;
      }
    }
    predecessor = constrainedState;
  }

  ConstrainedState successor;
  successor.pose = points.back();
  successor.distance = constrainedStates.back().distance;
  successor.maxVelocity = endVelocity;
  successor.minAcceleration = -1.0 * maxAcceleration;
  successor.minAcceleration = maxAcceleration;

  // Backward pass
  for (int i = points.size() - 1; i >= 0; i--) {
    auto& constrainedState = constrainedStates[i];
    QLength ds =
        constrainedState.distance - successor.distance;  // negative

    while (true) {
      // Enforce max velocity limit (reverse)
      // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
      QSpeed newMaxVelocity = 
          sqrt(successor.maxVelocity * successor.maxVelocity +
              successor.minAcceleration * ds * 2.0);

      // No more limits to impose! This state can be finalized.
      if (newMaxVelocity >= constrainedState.maxVelocity) {
        break;
      }

      constrainedState.maxVelocity = newMaxVelocity;

      // Check all acceleration constraints with the new max velocity.
      EnforceAccelerationLimits(reversed, constraints, &constrainedState);

      if (ds.convert(meter) > -kEpsilon) {
        break;
      }

      // If the actual acceleration for this state is lower than the min
      // acceleration, then we need to lower the min acceleration of the
      // successor and try again.
      QAcceleration actualAcceleration =
          (constrainedState.maxVelocity * constrainedState.maxVelocity -
           successor.maxVelocity * successor.maxVelocity) /
          (ds * 2.0);
      if (constrainedState.minAcceleration > actualAcceleration + 1E-6_mps2) {
        successor.minAcceleration = constrainedState.minAcceleration;
      } else {
        successor.minAcceleration = actualAcceleration;
        break;
      }
    }
    successor = constrainedState;
  }

  // Now we can integrate the constrained states forward in time to obtain our
  // trajectory states.

  std::vector<Trajectory::State> states(points.size());
  QTime t = 0_s;
  QLength s = 0_m;
  QSpeed v = 0_mps;

  for (unsigned int i = 0; i < constrainedStates.size(); i++) {
    auto state = constrainedStates[i];

    // Calculate the change in position between the current state and the
    // previous state.
    QLength ds = state.distance - s;

    // Calculate the acceleration between the current state and the previous
    // state.
    QAcceleration accel =
        (state.maxVelocity * state.maxVelocity - v * v) / (ds * 2);

    // Calculate dt.
    QTime dt = 0_s;
    if (i > 0) {
      states.at(i - 1).acceleration = reversed ? -accel : accel;
      if (fabs(accel.convert(mps2)) > 1E-6) {
        // v_f = v_0 + a * t
        dt = (state.maxVelocity - v) / accel;
      } else if (fabs(v.convert(mps)) > 1E-6) {
        // delta_x = v * t
        dt = ds / v;
      } else {
        printf("Something went wrong at iteration %d of time parameterization.\n",i);
        /*
        throw std::runtime_error(fmt::format(
            "Something went wrong at iteration {} of time parameterization.",
            i));
        */
        // Can't throw exceptions...
      }
    }

    v = state.maxVelocity;
    s = state.distance;

    t += dt;

    states[i] = {t, reversed ? -v : v, reversed ? -accel : accel,
                 state.pose.first, state.pose.second};
  }

  return Trajectory(states);
}

void TrajectoryParameterizer::EnforceAccelerationLimits(
    bool reverse,
    const std::vector<std::unique_ptr<TrajectoryConstraint>>& constraints,
    ConstrainedState* state) {
  for (auto&& constraint : constraints) {
    double factor = reverse ? -1.0 : 1.0;

    auto minMaxAccel = constraint->MinMaxAcceleration(
        state->pose.first, state->pose.second, state->maxVelocity * factor);

    if (minMaxAccel.minAcceleration > minMaxAccel.maxAcceleration) {
      printf("The constraint's min acceleration was greater than its max "
          "acceleration. To debug this, remove all constraints from the config "
          "and add each one individually. If the offending constraint was "
          "packaged with WPILib, please file a bug report.\n");
      /*
      throw std::runtime_error(
          "The constraint's min acceleration was greater than its max "
          "acceleration. To debug this, remove all constraints from the config "
          "and add each one individually. If the offending constraint was "
          "packaged with WPILib, please file a bug report.");
      */
      // Can't throw exceptions...
    }

    state->minAcceleration = mps2 * fmax(
        state->minAcceleration.convert(mps2),
        reverse ? -1.0 * minMaxAccel.maxAcceleration.convert(mps2) : minMaxAccel.minAcceleration.convert(mps2));

    state->maxAcceleration = mps2 * fmin(
        state->maxAcceleration.convert(mps2),
        reverse ? -1.0 * minMaxAccel.minAcceleration.convert(mps2) : minMaxAccel.maxAcceleration.convert(mps2));
  }
}