// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <thread>
#include <vector>

#include "vex.h"
#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/pid/PIDController.h"

namespace vpi {
/**
 * A class for programming and driving differential drive/skid-steer drive 
 * platforms, generally only for Autonomous routines.  This class contains 
 * both the DifferentialDrive and the DifferentialDriveOdometry.
 *
 * These drive bases typically have drop-center / skid-steer with two or more
 * wheels per side (e.g., 4WD or 6WD). This class takes a MotorController per
 * side. For four and six motor chassis, construct and pass in
 * MotorControllerGroup instances as follows.
 *
 * Four motor chassis:
 *
 * TODO - Fix documentation!!!!
 *
 * @code{.cpp}
 * void autonomous(void){
 *   // The motor and motor_group lines could all be
 *   // global variables.
 *   vex::motor Motor1 = motor(PORT1, ratio18_1, false);
 *   vex::motor Motor2 = motor(PORT2, ratio18_1, false);
 *   vex::motor_group m_left{Motor1, Motor2};
 *
 *   vex::motor Motor3 = motor(PORT3, ratio18_1, false);
 *   vex::motor Motor4 = motor(PORT4, ratio18_1, false);
 *   vex::motor_group m_right{Motor3, Motor4};
 *
 *   vpi::QLength trackWidth = 17_in;
 *   vpi::QLength wheelDiameter = 3.25_in;
 *
 *   vpi::DifferentialDriveChassisPID m_chassis(m_left, m_right,
 *                                              trackWidth, wheelDiameter,
 *                                              1.0, 1.0, 1.0); // Tune PID
 * }
 * @endcode
 *
 * A differential drive robot has left and right wheels separated by an
 * arbitrary width.
 *
 * Drive base diagram:
 * <pre>
 * |_______|
 * | |   | |
 *   |   |
 * |_|___|_|
 * |       |
 * </pre>
 *
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * The positive X axis points ahead, the positive Y axis points to the right,
 * and the positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 *
 * Inputs smaller then 0.02 will be set to 0, and larger values will be scaled
 * so that the full range is still used. This deadband value can be changed
 * with SetDeadband().
 */
  class DifferentialDriveChassisPID : public DifferentialDriveChassis {
    public:
      /**
      * Construct a DifferentialDrive that uses a PID for DriveToPoint.
      *
      * A motor_group can contain one or more motors. If a motor
      * needs to be inverted, do so before passing it in.
      *
      * @param leftMotorGroup - the left side motors
      *
      * @param rightMotorGroup - the right side motors
      *
      * @param driveTrackWidth - the distance between the RHS motors
      *                     and the LHS motors
      *
      * @param driveWheelDiameter - the size of the wheel, likely
      *                     3.25 inches or 4.0 inches (or close)
      *
      * @param pDistance - The PIDF coefficients for the distance controller
      *
      * @param pAngle - The PIDF coefficients for the angle controller
      *
      * @param pTurn - The PIDF coefficients for the turn controller
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
      DifferentialDriveChassisPID(vex::motor_group& leftMotorGroup, 
                               vex::motor_group& rightMotorGroup,
                               QLength driveTrackWidth,
                               QLength driveWheelDiameter,
                               PIDFParameters pDistance,
                               PIDFParameters pAngle,
                               PIDFParameters pTurn,
                               vex::gearSetting gs = vex::ratio18_1,
                               double gearRatio = 1.0,
                               vex::brakeType brake = vex::brakeType::brake);

    virtual void Reset();

    virtual void TurnToPoint(Point2d target, 
                      QAngularSpeed turnSpeed,
                      bool waitForCompletion=true) override;

    virtual void DriveDistance(QLength target, 
                        QSpeed movementSpeed,
                        bool waitForCompletion=true) override;

    virtual void DriveToPoint(Point2d target, QSpeed movementSpeed,
                        bool waitForCompletion=true) override;

    protected:
    PIDController* m_pidDistanceController = NULL;
    PIDController* m_pidAngleController = NULL;
    PIDController* m_pidTurnController = NULL;
    Point2d* m_targetPoint = NULL;
    Point2d* m_startPoint = NULL; // So we know if we overshot the target
    QSpeed* m_targetSpeed = NULL;
    QAngularSpeed* m_targetAngularSpeed = NULL;
    DifferentialDriveWheelSpeeds* m_desiredWheelSpeeds = NULL;
    vex::mutex m_mutex;

    virtual QLength DistanceToTarget();
    virtual QAngle AngleToTarget();

    virtual void ConsumeDistancePID(double v);
    virtual void ConsumeAnglePID(double a);
    virtual void ConsumeTurnPID(double a); 
  };
} // namespace vpi
