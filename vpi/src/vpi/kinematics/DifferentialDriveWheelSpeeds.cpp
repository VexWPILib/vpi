// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/units/QSpeed.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"

using namespace vpi;

void DifferentialDriveWheelSpeeds::Normalize(
    QSpeed attainableMaxSpeed) {
  double realMaxSpeed =
      fmax(fabs(left.convert(mps)), fabs(right.convert(mps)));

  if (realMaxSpeed > attainableMaxSpeed.convert(mps)) {
    left = (left.convert(mps) / realMaxSpeed * attainableMaxSpeed.convert(mps)) * mps;
    right = (right.convert(mps) / realMaxSpeed * attainableMaxSpeed.convert(mps)) * mps;
  }
}