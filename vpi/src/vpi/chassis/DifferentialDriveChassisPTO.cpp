// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vex.h"
#include "vpi/chassis/DifferentialDriveChassisPTO.h"

namespace vpi {
  DifferentialDriveChassisPTO::DifferentialDriveChassisPTO(vex::motor_group& fullLeftMotorGroup, 
                          vex::motor_group& fullRightMotorGroup, 
                          vex::motor_group& partialLeftMotorGroup, 
                          vex::motor_group& partialRightMotorGroup, 
                          QLength driveTrackWidth, QLength driveWheelDiameter,
                          vex::gearSetting gs,
                          double gearRatio, vex::brakeType brake,
                          bool fullPTO) : 
                  DifferentialDriveChassis(partialLeftMotorGroup, partialRightMotorGroup, driveTrackWidth, driveWheelDiameter,
                              gs, gearRatio, brake),
                  m_drivetrainFullPTO(driveTrackWidth, driveWheelDiameter, 
                                      fullLeftMotorGroup, fullRightMotorGroup),
                  m_drivetrainPartialPTO(driveTrackWidth, driveWheelDiameter, 
                                      partialLeftMotorGroup, partialRightMotorGroup),
                  m_FullPTO(fullPTO)
                  {
    m_drivetrainFullPTO.SetBrakeType(brake);
    m_drivetrainFullPTO.SetGearRatio(gearRatio);
    m_drivetrainPartialPTO.SetBrakeType(brake);
    m_drivetrainPartialPTO.SetGearRatio(gearRatio);
    if(m_FullPTO) {
      m_drivetrain = &m_drivetrainFullPTO;
    } else {
      m_drivetrain = &m_drivetrainPartialPTO;
    }
  }

}