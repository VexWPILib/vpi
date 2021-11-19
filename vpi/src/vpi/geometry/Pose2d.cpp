// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/units/QLength.h"
#include "vpi/geometry/Pose2d.h"

#include <cmath>

using namespace vpi;

Pose2d::Pose2d(Translation2d translation, Rotation2d rotation)
    : m_translation(translation), m_rotation(rotation) {}

Pose2d::Pose2d(QLength x, QLength y, Rotation2d rotation)
    : m_translation(x, y), m_rotation(rotation) {}

Pose2d Pose2d::operator+(const Transform2d& other) const {
  return TransformBy(other);
}

Transform2d Pose2d::operator-(const Pose2d& other) const {
  const auto pose = this->RelativeTo(other);
  return Transform2d(pose.Translation(), pose.Rotation());
}

bool Pose2d::operator==(const Pose2d& other) const {
  return m_translation == other.m_translation && m_rotation == other.m_rotation;
}

bool Pose2d::operator!=(const Pose2d& other) const {
  return !operator==(other);
}

Pose2d Pose2d::TransformBy(const Transform2d& other) const {
  return {m_translation + (other.Translation().RotateBy(m_rotation)),
          m_rotation + other.Rotation()};
}

Pose2d Pose2d::RelativeTo(const Pose2d& other) const {
  const Transform2d transform{other, *this};
  return {transform.Translation(), transform.Rotation()};
}

Pose2d Pose2d::Exp(const Twist2d& twist) const {
  const auto dx = twist.dx;
  const auto dy = twist.dy;
  const auto dtheta = twist.dtheta;

  const auto sinTheta = std::sin(dtheta.convert(radian));
  const auto cosTheta = std::cos(dtheta.convert(radian));

  double s, c;
  if (std::abs(dtheta.convert(radian)) < 1E-9) {
    s = 1.0 - 1.0 / 6.0 * dtheta.convert(radian) * dtheta.convert(radian);
    c = 0.5 * dtheta.convert(radian);
  } else {
    s = sinTheta / dtheta.convert(radian);
    c = (1 - cosTheta) / dtheta.convert(radian);
  }

  const Transform2d transform{Translation2d{dx * s - dy * c, dx * c + dy * s},
                              Rotation2d{cosTheta, sinTheta}};

  return *this + transform;
}

Twist2d Pose2d::Log(const Pose2d& end) const {
  const auto transform = end.RelativeTo(*this);
  const auto dtheta = transform.Rotation().ToAngle();
  const auto halfDtheta = dtheta / 2.0;

  const auto cosMinusOne = transform.Rotation().Cos() - 1;

  double halfThetaByTanOfHalfDtheta;

  if (std::abs(cosMinusOne) < 1E-9) {
    halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta.convert(radian) * dtheta.convert(radian);
  } else {
    halfThetaByTanOfHalfDtheta =
        -(halfDtheta.convert(radian) * transform.Rotation().Sin()) / cosMinusOne;
  }

  const Translation2d translationPart =
      transform.Translation().RotateBy(
          {halfThetaByTanOfHalfDtheta, -halfDtheta.convert(radian)}) *
      std::hypot(halfThetaByTanOfHalfDtheta, halfDtheta.convert(radian));

  Twist2d retval;
  retval.dx = translationPart.X();
  retval.dy = translationPart.Y();
  retval.dtheta = dtheta;
  return retval;
}

QLength Pose2d::DistanceTo(const Pose2d& end) const {
  Point2d pthis = Point2d(X(), Y());
  Point2d pend = Point2d(end.X(), end.Y());
  return pthis.computeDistanceToPoint(pend);
}

QLength Pose2d::DistanceTo(const Point2d& p) const {
  Point2d pthis = Point2d(X(), Y());
  return pthis.computeDistanceToPoint(p);
}

QAngle Pose2d::AngleTo(const Point2d& p) const {
  Point2d pthis = Point2d(X(), Y());
  QAngle headingToPoint = pthis.computeAngleToPoint(p);
  return UnitUtils::constrainTo180(headingToPoint - Rotation().ToAngle());
}