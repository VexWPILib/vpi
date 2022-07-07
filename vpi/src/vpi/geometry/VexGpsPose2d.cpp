// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vpi/geometry/VexGpsPose2d.h"

namespace vpi {
  VexGpsPose2d VexGpsPose2d::WeightedAverage(VexGpsPose2d a, VexGpsPose2d b, int minQuality) {
    // TODO - Validate the vector implementation and then call into that
    int aq = a.Quality();
    aq = aq - minQuality;
    if(aq < 0) {
      aq = 0;
    } else {
      aq = aq * aq;
    }
    double ax = a.X().convert(inch) * aq;
    double ay = a.Y().convert(inch) * aq;
    double ah = UnitUtils::constrainTo360(a.Theta()).convert(degree) * aq;
    QTime at = a.Timestamp() * aq;

    int bq = b.Quality();
    bq = bq - minQuality;
    if(aq < 0) {
      bq = 0;
    } else {
      bq = bq * bq;
    }
    double bx = b.X().convert(inch) * bq;
    double by = b.Y().convert(inch) * bq;
    double bh = UnitUtils::constrainTo360(b.Theta()).convert(degree) * bq;
    QTime bt = b.Timestamp() * bq;
    int cq = 0;
    if(aq + bq == 0) { 
      cq = 1;  // Prevent divide by zero
    }
    double mx = (ax + bx) / (aq + bq + cq);
    double my = (ay + by) / (aq + bq + cq);
    double mh = UnitUtils::constrainTo360((ah + bh) / (aq + bq + cq) * degree).convert(degree);
    int mq = minQuality + std::sqrt((aq + bq) / 2);
    QTime mt = (at + bt) / (aq + bq + cq);
    return VexGpsPose2d(mx * inch, my * inch, mh * degree, mq, mt);
  }

  VexGpsPose2d VexGpsPose2d::WeightedAverage(std::vector<VexGpsPose2d> v, int minQuality) {
    int tq = 0;
    double tx = 0;
    double ty = 0;
    double th = 0;
    QTime tt = 0_ms;
    for(VexGpsPose2d p : v) {
      int q = p.Quality();
      q = q - minQuality;
      if(q < 0) {
        q = 0;
      }
      tq += q;
      tx += p.X().convert(inch) * q;
      ty += p.Y().convert(inch) * q;
      th += UnitUtils::constrainTo360(p.Theta()).convert(degree) * q;
      tt = tt + p.Timestamp() * q;
    }
    int cq = 0;
    if(tq == 0) { 
      cq = 1;  // Prevent divide by zero
    }
    double mx = tx / (tq + cq);
    double my = ty / (tq + cq);
    double mh = UnitUtils::constrainTo360(th / (tq + cq) * degree).convert(degree);
    int mq = minQuality + std::sqrt(tq / v.size());
    QTime mt = tt / (tq + cq);
    if(tq == 0) {
      mq = 0;
    }
    return VexGpsPose2d(mx * inch, my * inch, mh * degree, mq, mt);
  }

  VexGpsPose2d VexGpsPose2d::Average(VexGpsPose2d a, VexGpsPose2d b) {
    double ax = a.X().convert(inch);
    double ay = a.Y().convert(inch);
    double ah = UnitUtils::constrainTo360(a.Theta()).convert(degree);

    double bx = b.X().convert(inch);
    double by = b.Y().convert(inch);
    double bh = UnitUtils::constrainTo360(b.Theta()).convert(degree);

    double mx = (ax + bx) / 2;
    double my = (ay + by) / 2;
    double mh = UnitUtils::constrainTo360((ah + bh) / 2 * degree).convert(degree);
    int mq = (a.Quality() + b.Quality()) / 2;
    return VexGpsPose2d(mx * inch, my * inch, mh * degree, mq);
  }

  VexGpsPose2d VexGpsPose2d::MedianOfThree(VexGpsPose2d a, VexGpsPose2d b, VexGpsPose2d c) {
    double ax = a.X().convert(inch);
    double ay = a.Y().convert(inch);
    double ah = UnitUtils::constrainTo180(a.Theta()).convert(degree);
    int aq = a.Quality();
    double bx = b.X().convert(inch);
    double by = b.Y().convert(inch);
    double bh = UnitUtils::constrainTo180(b.Theta()).convert(degree);
    int bq = b.Quality();
    double cx = b.X().convert(inch);
    double cy = b.Y().convert(inch);
    double ch = UnitUtils::constrainTo180(b.Theta()).convert(degree);
    int cq = c.Quality();
    if(fabs(ah) > 135 || fabs(bh) > 135 || fabs(ch) > 135) {
      ah = UnitUtils::constrainTo360(a.Theta()).convert(degree);
      bh = UnitUtils::constrainTo360(b.Theta()).convert(degree);
      ch = UnitUtils::constrainTo360(c.Theta()).convert(degree);
    }
    double mx = fmax(fmin(ax,bx), fmin(fmax(ax,bx),cx));
    double my = fmax(fmin(ay,by), fmin(fmax(ay,by),cy));
    double mh = fmax(fmin(ah,bh), fmin(fmax(ah,bh),ch));
    int mq = fmax(fmin(aq,bq), fmin(fmax(aq,bq),cq));
    return VexGpsPose2d(mx * inch, my * inch, mh * degree, mq);
  }

  VexGpsPose2d VexGpsPose2d::MedianOfFive(VexGpsPose2d a, VexGpsPose2d b, VexGpsPose2d c,
              VexGpsPose2d d, VexGpsPose2d e) {
    VexGpsPose2d x = VexGpsPose2d::MedianOfThree(a,b,c);
    VexGpsPose2d y = VexGpsPose2d::MedianOfThree(b,c,d);
    VexGpsPose2d z = VexGpsPose2d::MedianOfThree(c,d,e);
    return VexGpsPose2d::MedianOfThree(x,y,z);
  }

  std::vector<double> VexGpsPose2d::CircleLineIntersection(QLength r, VexGpsPose2d p0, VexGpsPose2d p1) {
    std::vector<double> retval;
    // Based on https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf
    Vector2d d = Vector2d(p1.X().convert(meter) - p0.X().convert(meter), p1.Y().convert(meter) - p0.Y().convert(meter));
    Vector2d f = Vector2d(p0.X().convert(meter) - X().convert(meter), p0.Y().convert(meter) - Y().convert(meter));

    double a = d.Dot(d);
    double b = 2 * f.Dot(d);
    double c = f.Dot(f) - std::pow(r.convert(meter), 2);
    double discriminant = std::pow(b, 2) - (4.0 * a * c);
    if(discriminant > 0) {
      // At least 1 solution exists
      double sd = std::sqrt(discriminant);
      double t1 = (-b - sd) / (2.0 * a);
      double t2 = (-b + sd) / (2.0 * a);
      if(t2 >= 0 && t2 <= 1.0) {
        retval.emplace_back(t2);
      }
      if(t1 >= 0 && t1 <= 1.0) {
        retval.emplace_back(t1);
      }
    }
    return retval;
  }

  VexGpsPose2d VexGpsPose2d::CircleLineIntersectionPoint(QLength r, VexGpsPose2d p0, VexGpsPose2d p1, double t) {
    VexGpsPose2d retval = p1;
    // Based on https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf
    double dx = p1.X().convert(meter) - p0.X().convert(meter);
    double dy = p1.Y().convert(meter) - p0.Y().convert(meter);
    Vector2d d = Vector2d(dx, dy);
    Vector2d f = Vector2d(p0.X().convert(meter) - X().convert(meter), p0.Y().convert(meter) - Y().convert(meter));

    double a = d.Dot(d);
    double b = 2 * f.Dot(d);
    double c = f.Dot(f) - std::pow(r.convert(meter), 2);
    double discriminant = std::pow(b, 2) - (4.0 * a * c);

    if(discriminant > 0) {
      // At least 1 solution exists
      double newx = p0.X().convert(meter) + t * dx; 
      double newy = p0.Y().convert(meter) + t * dy;
      retval = VexGpsPose2d(newx * meter, newy * meter, 0 * radian);
    }
    return retval;
  }

  VexGpsPose2d::PointRelativeOrientation VexGpsPose2d::OrientationOfPoseAndPoint(VexGpsPose2d p) {
    double p1x = X().convert(meter);
    double p1y = Y().convert(meter);
    double p2x = p1x + cos(Theta().convert(radian));
    double p2y = p1y + sin(Theta().convert(radian));
    double px = p.X().convert(meter);
    double py = p.Y().convert(meter);

    double side = (p2y - p1y) * (px - p2x) - (p2x - p1x) * (py - p2y);
    if(fabs(side) < 1e-9) {
      return VexGpsPose2d::PointRelativeOrientation::STRAIGHT_AHEAD;
    } else if (side > 0) {
      return VexGpsPose2d::PointRelativeOrientation::LEFT;
    } else {
      return VexGpsPose2d::PointRelativeOrientation::RIGHT;
    }
  }

  QCurvature VexGpsPose2d::CurvatureToPoint(VexGpsPose2d p, QLength r) {
    double dx = X().convert(meter) - p.X().convert(meter);
    double c = fabs(2.0 * dx / std::pow(r.convert(meter),2));
    if(OrientationOfPoseAndPoint(p) == VexGpsPose2d::PointRelativeOrientation::LEFT) {
      c = -1.0 * c;
    }
    QCurvature retval(c);

    return retval;
  }


}