// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <thread>
#include <vector>

#include "vex.h"
#include "vpi/chassis/DifferentialDriveChassis.h"

namespace vpi {
/**
 * A class for programming and driving differential drive/skid-steer drive 
 * platforms, generally only for Autonomous routines.  This class is
 * basically just like the @see DifferentialDriveChassis but with two
 * modes - one with the first set of MotorGroups and the other with the
 * second set of MotorGroups.
 */
  class DifferentialDriveChassisPTO : public DifferentialDriveChassis {
    public:
      /**
      * Construct a DifferentialDrivePTO.
      *
      * A motor_group can contain one or more motors. If a motor
      * needs to be inverted, do so before passing it in.
      *
      * @param fullLeftMotorGroup - the left side motors when all
      *                             motors are engaged in drivetrain
      *
      * @param fullRightMotorGroup - the right side motors when all
      *                             motors are engaged in drivetrain
      *
      * @param partialLeftMotorGroup - the left side motors when not all
      *                             motors are engaged in drivetrain
      *
      * @param partialRightMotorGroup - the right side motors not all
      *                             motors are engaged in drivetrain
      *
      * @param driveTrackWidth - the distance between the RHS motors
      *                     and the LHS motors
      *
      * @param driveWheelDiameter - the size of the wheel, likely
      *                     3.25 inches or 4.0 inches (or close)
      *
      * @param gs - The gearset of the motors (vex::ratio36_1 or
      *             vex::ratio18_1 or vex::ratio6_1) defaults to
      *             18_1 which is the green cartridge
      *
      * @param gearRatio  - Any external gearing from the motors
      *                     to the powered wheels. Defaults to 1.0
      *
      * @param brake      - coast/brake/hold for the drive wheels
      *                   defaults to brake
      *
      */
      DifferentialDriveChassisPTO(vex::motor_group& fullLeftMotorGroup, 
                               vex::motor_group& fullRightMotorGroup,
                               vex::motor_group& partialLeftMotorGroup,
                               vex::motor_group& partialRightMotorGroup,
                               QLength driveTrackWidth,
                               QLength driveWheelDiameter,
                               vex::gearSetting gs = vex::ratio18_1,
                               double gearRatio = 1.0,
                               vex::brakeType brake = vex::brakeType::brake,
                               bool m_FullPTO = true);

    virtual void Reset();

    virtual void TurnToPoint(VexGpsPose2d target, 
                      QAngularSpeed turnSpeed,
                      bool waitForCompletion=true) override;

    virtual void DriveDistance(QLength target, 
                        QSpeed movementSpeed,
                        bool waitForCompletion=true) override;

    virtual void DriveToPoint(VexGpsPose2d target, QSpeed movementSpeed,
                        bool waitForCompletion=true) override;

    virtual ~DifferentialDriveChassisPTO() {
      DisableOdom();
    }
    
    virtual void SetFullPTO(bool fullPTO) {
      if(fullPTO) {
        m_FullPTO = true;
        m_drivetrain = &m_drivetrainFullPTO;
      } else {
        m_FullPTO = false;
        m_drivetrain = &m_drivetrainPartialPTO;
      }
    }

    protected:
      DifferentialDrive m_drivetrainFullPTO;
      DifferentialDrive m_drivetrainPartialPTO;
      bool m_FullPTO;

  };
} // namespace vpi
