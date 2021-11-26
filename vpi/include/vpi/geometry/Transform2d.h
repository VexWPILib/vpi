// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "vpi/units/QLength.h"
#include "vpi/geometry/Translation2d.h"

namespace vpi {

class Pose2d;

/**
 * Represents a transformation for a Pose2d.
 */
class Transform2d {
 public:
  /**
   * Constructs the transform that maps the initial pose to the final pose.
   *
   * @param initial The initial pose for the transformation.
   * @param f The final pose for the transformation.
   */
  Transform2d(Pose2d initial, Pose2d f);

  /**
   * Constructs a transform with the given translation and rotation components.
   *
   * @param translation Translational component of the transform.
   * @param r Rotational component of the transform.
   */
  Transform2d(Translation2d translation, Rotation2d r);

  /**
   * Constructs the identity transform -- maps an initial pose to itself.
   */
  constexpr Transform2d() = default;

  /**
   * Returns the translation component of the transformation.
   *
   * @return Reference to the translational component of the transform.
   */
  const Translation2d& Translation() const { return m_translation; }

  /**
   * Returns the X component of the transformation's translation.
   *
   * @return The x component of the transformation's translation.
   */
  QLength X() const { return m_translation.X(); }

  /**
   * Returns the Y component of the transformation's translation.
   *
   * @return The y component of the transformation's translation.
   */
  QLength Y() const { return m_translation.Y(); }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return Reference to the rotational component of the transform.
   */
  const Rotation2d& Rotation() const { return m_rotation; }

  /**
   * Invert the transformation. This is useful for undoing a transformation.
   *
   * @return The inverted transformation.
   */
  Transform2d Inverse() const;

  /**
   * Scales the transform by the scalar.
   *
   * @param scalar The scalar.
   * @return The scaled Transform2d.
   */
  Transform2d operator*(double scalar) const {
    return Transform2d(m_translation * scalar, m_rotation * scalar);
  }

  /**
   * Composes two transformations.
   *
   * @param other The transform to compose with this one.
   * @return The composition of the two transformations.
   */
  Transform2d operator+(const Transform2d& other) const;

  /**
   * Checks equality between this Transform2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Transform2d& other) const;

  /**
   * Checks inequality between this Transform2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Transform2d& other) const;

 private:
  Translation2d m_translation;
  Rotation2d m_rotation;
};
}  // namespace vpi
