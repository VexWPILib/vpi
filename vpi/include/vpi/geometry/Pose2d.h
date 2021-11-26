// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/units/QLength.h"
#include "vpi/units/QCurvature.h"
#include "vpi/geometry/Point2d.h"
#include "vpi/geometry/Transform2d.h"
#include "vpi/geometry/Translation2d.h"
#include "vpi/geometry/Twist2d.h"


namespace vpi {

// Forward declare VexGpsPose2d to avoid circular dependencies
class VexGpsPose2d;

/**
 * Represents a 2d pose containing translational and rotational elements.
 *
 * When using Odometry for VRC field co-ordinates, @see VexGpsPose2d
 *
 * @see VexGpsPose2d
 */
class Pose2d {
 public:
  // Conversions between Pose2d and VexGpsPose2d
  // -------------------------------------------
  // Constructor
  Pose2d(const VexGpsPose2d& v);
  // Assignment
  Pose2d& operator=(const VexGpsPose2d& v);
   // Type-cast
  operator VexGpsPose2d();
  // -------------------------------------------

  /**
   * Constructs a pose at the origin facing toward the positive X axis.
   * (Translation2d{0, 0} and Rotation{0})
   */
  constexpr Pose2d() = default;

  /**
   * Constructs a pose with the specified translation and rotation.
   *
   * @param translation The translational component of the pose.
   * @param r The rotational component of the pose.
   */
  Pose2d(Translation2d translation, Rotation2d r);

  /**
   * Convenience constructors that takes in x and y values directly instead of
   * having to construct a Translation2d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param r The rotational component of the pose.
   */
  Pose2d(QLength x, QLength y, Rotation2d r);

  /**
   * Transforms the pose by the given transformation and returns the new
   * transformed pose.
   *
   * [x_new]    [cos, -sin, 0][transform.x]
   * [y_new] += [sin,  cos, 0][transform.y]
   * [t_new]    [0,    0,   1][transform.t]
   *
   * @param other The transform to transform the pose by.
   *
   * @return The transformed pose.
   */
  Pose2d operator+(const Transform2d& other) const;

  /**
   * Returns the Transform2d that maps the one pose to another.
   *
   * @param other The initial pose of the transformation.
   * @return The transform that maps the other pose to the current pose.
   */
  Transform2d operator-(const Pose2d& other) const;

  /**
   * Checks equality between this Pose2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Pose2d& other) const;

  /**
   * Checks inequality between this Pose2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Pose2d& other) const;

  /**
   * Returns the underlying translation.
   *
   * @return Reference to the translational component of the pose.
   */
  const Translation2d& Translation() const { return m_translation; }

  /**
   * Returns the X component of the pose's translation.
   *
   * @return The x component of the pose's translation.
   */
  QLength X() const { return m_translation.X(); }

  /**
   * Returns the Y component of the pose's translation.
   *
   * @return The y component of the pose's translation.
   */
  QLength Y() const { return m_translation.Y(); }

  /**
   * Returns the underlying rotation.
   *
   * @return Reference to the rotational component of the pose.
   */
  const Rotation2d& Rotation() const { return m_rotation; }

  /**
   * Transforms the pose by the given transformation and returns the new pose.
   * See + operator for the matrix multiplication performed.
   *
   * @param other The transform to transform the pose by.
   *
   * @return The transformed pose.
   */
  Pose2d TransformBy(const Transform2d& other) const;

  /**
   * Returns the other pose relative to the current pose.
   *
   * This function can often be used for trajectory tracking or pose
   * stabilization algorithms to get the error between the reference and the
   * current pose.
   *
   * @param other The pose that is the origin of the new coordinate frame that
   * the current pose will be converted into.
   *
   * @return The current pose relative to the new origin pose.
   */
  Pose2d RelativeTo(const Pose2d& other) const;

  /**
   * Obtain a new Pose2d from a (constant curvature) velocity.
   *
   * See https://file.tavsys.net/control/controls-engineering-in-frc.pdf section
   * 10.2 "Pose exponential" for a derivation.
   *
   * The twist is a change in pose in the robot's coordinate frame since the
   * previous pose update. When the user runs exp() on the previous known
   * field-relative pose with the argument being the twist, the user will
   * receive the new field-relative pose.
   *
   * "Exp" represents the pose exponential, which is solving a differential
   * equation moving the pose forward in time.
   *
   * @param twist The change in pose in the robot's coordinate frame since the
   * previous pose update. For example, if a non-holonomic robot moves forward
   * 0.01 meters and changes angle by 0.5 degrees since the previous pose
   * update, the twist would be Twist2d{0.01, 0.0, toRadians(0.5)}
   *
   * @return The new pose of the robot.
   */
  Pose2d Exp(const Twist2d& twist) const;

  /**
   * Returns a Twist2d that maps this pose to the end pose. If c is the output
   * of a.Log(b), then a.Exp(c) would yield b.
   *
   * @param end The end pose for the transformation.
   *
   * @return The twist that maps this to end.
   */
  Twist2d Log(const Pose2d& end) const;

  /**
   * Returns the distance to the specified pose.
   *
   * @param end The end pose
   *
   * @return The QLength to this pose
   */
  QLength DistanceTo(const Pose2d& end) const;

  /**
   * Returns the distance to the specified pose.
   *
   * @param end The end pose
   *
   * @return The QLength to this pose
   */
  QLength DistanceTo(const Point2d& p) const;

  /**
   * Returns the angle to the specified point
   *
   * @param p the point
   *
   * @return The QAngle to this point
   */
  QAngle AngleTo(const Point2d& p) const;

 private:
  Translation2d m_translation;
  Rotation2d m_rotation;
};

struct Pose2dWithCurvature {
  Pose2d pose;
  QCurvature curvature;

  Pose2dWithCurvature(Pose2d p, QCurvature c) : pose(p), curvature(c) {}
  Pose2dWithCurvature() : pose({0_m, 0_m}, 0_deg),
                          curvature() {}
};

}  // namespace vpi
