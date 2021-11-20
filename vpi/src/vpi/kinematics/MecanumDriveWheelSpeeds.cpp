// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/units/QSpeed.h"
#include "vpi/kinematics/MecanumDriveWheelSpeeds.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace vpi;

void MecanumDriveWheelSpeeds::Normalize(
    QSpeed attainableMaxSpeed) {
  std::array<QSpeed, 4> wheelSpeeds{frontLeft, frontRight,
                                    rearLeft, rearRight};
  QSpeed realMaxSpeed = *std::max_element(
      wheelSpeeds.begin(), wheelSpeeds.end(), [](const QSpeed& a, const QSpeed& b) {
        return fabs(a.convert(mps)) < fabs(b.convert(mps));
      });

  if (fabs(realMaxSpeed.convert(mps)) > attainableMaxSpeed.convert(mps)) {
    for (int i = 0; i < 4; ++i) {
      wheelSpeeds[i] = wheelSpeeds[i] / fabs(realMaxSpeed.convert(mps)) * attainableMaxSpeed.convert(mps);
    }
    frontLeft = wheelSpeeds[0];
    frontRight = wheelSpeeds[1];
    rearLeft = wheelSpeeds[2];
    rearRight = wheelSpeeds[3];
  }
}