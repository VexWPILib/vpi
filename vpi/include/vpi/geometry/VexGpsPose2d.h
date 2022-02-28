// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <vector>

#include "vpi/geometry/Pose2d.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {

 /**
  * WPILib assumes a coordinate system where increasing X on the 
  * horizontal axis would have a heading of 0 degrees, while the
  * Vex GPS sensor reports this as a heading of 90 degrees.  Given that
  * users would want to use their familiar co-ordinates and heading,
  * we will default to VEX_GPS, and ensure that all calls inside this
  * library use VPI_NATIVE when creating Pose2d objects.
  */
class VexGpsPose2d {
 public:
  /**
   * Returns the quality-based weighted average of 2 positions.
   */
  static VexGpsPose2d WeightedAverage(VexGpsPose2d a, VexGpsPose2d b, int minQuality=90);

  /**
   * Returns the quality-based weighted average of a vector of positions.
   */
  static VexGpsPose2d WeightedAverage(std::vector<VexGpsPose2d> v, int minQuality=90);

  /**
   * Returns the simple average of 2 positions.
   */
  static VexGpsPose2d Average(VexGpsPose2d a, VexGpsPose2d b);

  /**
   * Returns the median of 3 poses.  Useful for filtering
   */
  static VexGpsPose2d MedianOfThree(VexGpsPose2d a, VexGpsPose2d b, VexGpsPose2d c);

  /**
   * Returns the median of 5 poses.  Useful for filtering
   */
  static VexGpsPose2d MedianOfFive(VexGpsPose2d a, VexGpsPose2d b, VexGpsPose2d c,
                                      VexGpsPose2d d, VexGpsPose2d e);

  /**
   * Constructs a pose at the origin facing toward the positive X axis.
   * (Translation2d{0, 0} and Rotation{0})
   */
  constexpr VexGpsPose2d() = default;

  /**
   * Convenience constructors that takes in x and y values directly instead of
   * having to construct a Translation2d.
   *
   * @param x The x component of the translational component of the pose.
   * @param y The y component of the translational component of the pose.
   * @param t The heading component of the pose.
   * @param q The quality of the reading
   * @param ts The timestamp of the reading
   */
  VexGpsPose2d(QLength x, QLength y, QAngle t, int q=100, QTime ts=0_ms) : 
    m_x(x), m_y(y), m_theta(t), m_quality(q), m_timestamp(ts) {}

  // Conversions between Pose2d and VexGpsPose2d
  // -------------------------------------------
  // Constructor
  VexGpsPose2d(const Pose2d& p) : m_x(p.X()), m_y(p.Y()),
      m_theta(UnitUtils::constrainTo180(90_deg - p.Rotation().ToAngle())), m_quality(100) {}

  // Assignment
  VexGpsPose2d& operator=(const Pose2d& p) {
    this->m_x = p.X();
    this->m_y = p.Y();
    this->m_theta = UnitUtils::constrainTo180(90_deg - p.Rotation().ToAngle());
    return *this;
  }
   // Type-cast
  operator Pose2d() {return Pose2d(m_x, m_y, UnitUtils::constrainTo180(90_deg - m_theta));}

  const QLength X() const {return m_x;}
  const QLength Y() const {return m_y;}
  const QAngle Theta() const {return m_theta;}
  const int Quality() const {return m_quality;}
  const QTime Timestamp() const {return m_timestamp;}

  /**
   * Returns the distance to the specified pose.
   *
   * @param end The end pose
   *
   * @return The QLength to this pose
   */
  QLength DistanceTo(const VexGpsPose2d& end) const {
    Point2d pthis = Point2d(X(), Y());
    Point2d pend = Point2d(end.X(), end.Y());
    return pthis.computeDistanceToPoint(pend);
  }

  /**
   * Returns the angle to the specified point
   *
   * @param p the point
   *
   * @return The QAngle to this point
   */
  QAngle AngleTo(const VexGpsPose2d& p) const {
    QAngle a = std::atan2((p.X() - X()).convert(inch), (p.Y() - Y()).convert(inch)) * radian;
    return UnitUtils::constrainTo180(a);
  }

 private:
  QLength m_x = 0_m;
  QLength m_y = 0_m;
  QAngle m_theta = 0_rad;
  int m_quality = 100;
  QTime m_timestamp = 0_ms;
};
} // namespace vpi