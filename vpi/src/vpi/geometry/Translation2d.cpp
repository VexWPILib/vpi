// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/geometry/Translation2d.h"
#include <cmath>

using namespace vpi;

Translation2d::Translation2d(QLength x, QLength y)
    : m_x(x), m_y(y) {}

Translation2d::Translation2d(QLength distance, const Rotation2d& angle)
    : m_x(distance * angle.Cos()), m_y(distance * angle.Sin()) {}

QLength Translation2d::Distance(const Translation2d& other) const {
  return std::hypot(other.m_x.convert(meter) - m_x.convert(meter), other.m_y.convert(meter) - m_y.convert(meter)) * meter;
}

QLength Translation2d::Norm() const {
  return std::hypot(m_x.convert(meter), m_y.convert(meter)) * meter;
}

Translation2d Translation2d::RotateBy(const Rotation2d& other) const {
  return {m_x * other.Cos() - m_y * other.Sin(),
          m_x * other.Sin() + m_y * other.Cos()};
}

Translation2d Translation2d::operator+(const Translation2d& other) const {
  return {X() + other.X(), Y() + other.Y()};
}

Translation2d Translation2d::operator-(const Translation2d& other) const {
  return *this + -other;
}

Translation2d Translation2d::operator-() const {
  return {-1.0 * m_x, -1.0 * m_y};
}

Translation2d Translation2d::operator*(double scalar) const {
  return {scalar * m_x, scalar * m_y};
}

Translation2d Translation2d::operator/(double scalar) const {
  return *this * (1.0 / scalar);
}

bool Translation2d::operator==(const Translation2d& other) const {
  return fabs(m_x.convert(meter) - other.m_x.convert(meter)) < 1E-9 &&
         fabs(m_y.convert(meter) - other.m_y.convert(meter)) < 1E-9;
}

bool Translation2d::operator!=(const Translation2d& other) const {
  return !operator==(other);
}
