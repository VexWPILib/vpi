// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <thread>
#include <vector>

#include "vex.h"
#include "vpi/drive/DifferentialDrive.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"
#include "vpi/hal/gps/VexGpsPose2dGPS.h"
#include "vpi/kinematics/DifferentialDriveOdometry.h"
#include "vpi/log/Logger.h"
#include "vpi/units/UnitUtils.h"

namespace vpi {
/**
 * A class for programming and driving differential drive/skid-steer drive 
 * platforms.  This class contains both the DifferentialDrive and the
 * DifferentialDriveOdometry
 *
 * These drive bases typically have drop-center / skid-steer with two or more
 * wheels per side (e.g., 4WD or 6WD). This class takes a MotorController per
 * side. For four and six motor chassis, construct and pass in
 * MotorControllerGroup instances as follows.
 *
 * Four motor chassis:
 *
 * @code{.cpp}
 * void usercontrol(void){
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
 *   vpi::DifferentialDriveChassis m_chassis(m_left, m_right,
 *                                           trackWidth, wheelDiameter);
 *   vex::controller c;
 *   while(1) {
 *      double a1 = (double)c.Axis1.position(vex::percentUnits::pct);
 *      double a2 = (double)c.Axis2.position(vex::percentUnits::pct);
 *      double a3 = (double)c.Axis3.position(vex::percentUnits::pct);
 *      m_chassis.ArcadeDrive(a3,a1);
 *      // m_chassis.TankDrive(a3,a2);
 *      wait(20, vex::msec);
 *   }
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
 * Prioritizes:
 * 1. The first GPS sensor with quality above GPS_SENSOR_RM2_ERROR_THRESHOLD.
 * 2. For heading, averaged value of all inertial sensors configured.
 * 3. If no GPS and no inertial sensors, heading computed from Tracking Wheels
 * 4. Distance computed by Tracking Wheels, if present.
 * 5. If no Tracking Wheels, uses drive train wheels for heading and distance.
 *
 * Inputs smaller then 0.02 will be set to 0, and larger values will be scaled
 * so that the full range is still used. This deadband value can be changed
 * with SetDeadband().
 */
  class DifferentialDriveChassis {
    public:
      enum MultipleGpsHandler {
        FIRST_ABOVE_QUALITY,
        HIGHEST_QUALITY,
        QUALITY_WEIGHTEDAVERAGE
      };

      /**
      * Construct a DifferentialDrive.
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
      DifferentialDriveChassis(vex::motor_group& leftMotorGroup, 
                               vex::motor_group& rightMotorGroup,
                               QLength driveTrackWidth,
                               QLength driveWheelDiameter,
                               vex::gearSetting gs = vex::ratio18_1,
                               double gearRatio = 1.0,
                               vex::brakeType brake = vex::brakeType::brake);

      virtual ~DifferentialDriveChassis() {
        DisableOdom();
      }

      DifferentialDriveChassis(DifferentialDriveChassis&&) = default;
      DifferentialDriveChassis& operator=(DifferentialDriveChassis&&) = default;
      

      /**
      * Arcade drive method for differential drive platform.
      *
      * Note: Some drivers may prefer inverted rotation controls. This can be done
      * by negating the value passed for rotation.
      *
      * @param xSpeed        The speed at which the robot should drive along the X
      *                      axis [-1.0..1.0]. Forward is positive.
      * @param zRotation     The rotation rate of the robot around the Z axis
      *                      [-1.0..1.0]. Clockwise is positive.
      * @param squareInputs If set, decreases the input sensitivity at low speeds.
      */
      void ArcadeDrive(double xSpeed, double zRotation, bool squareInputs = false) {
        m_drivetrain->ArcadeDrive(xSpeed, zRotation, squareInputs);
      }

      /**
       * Curvature drive method for differential drive platform.
       *
       * The rotation argument controls the curvature of the robot's path rather
       * than its rate of heading change. This makes the robot more controllable at
       * high speeds.  Allows turn-in-place if forward speed is zero.
       *
       * @param xSpeed           The robot's speed along the X axis [-1.0..1.0].
       *                         Forward is positive.
       * @param zRotation        The normalized curvature [-1.0..1.0]. Clockwise is
       *                         positive.
       */
      void CurvatureDrive(double xSpeed, double zRotation) {
        m_drivetrain->CurvatureDrive(xSpeed, zRotation);
      }
      
      /**
      * Tank drive method for differential drive platform.
      *
      * @param leftSpeed     The robot left side's speed along the X axis
      *                      [-1.0..1.0]. Forward is positive.
      * @param rightSpeed    The robot right side's speed along the X axis
      *                      [-1.0..1.0]. Forward is positive.
      * @param squareInputs If set, decreases the input sensitivity at low speeds.
      */
      void TankDrive(double leftSpeed, double rightSpeed, bool squareInputs = false) {
        m_drivetrain->TankDrive(leftSpeed, rightSpeed, squareInputs);
      }

      virtual void ResetPosition(const VexGpsPose2d& pose, const Rotation2d& gyroAngle) {
        m_odometry.ResetPosition(pose, gyroAngle);
        SafeOdomReset();
      }

      virtual void ResetPosition(const VexGpsPose2d& pose) {
        m_odometry.ResetPosition(pose);
        SafeOdomReset();
      }

      /**
      * Enables odometry; call this one time during autonomous
      *
      * @code{.cpp}
      * void autonomous(void) {
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
      *   vpi::DifferentialDriveChassis m_chassis{m_left, m_right};
      *   vpi::Pose2d p; // Set to initial location
      *   vpi::Rotation2d h; // Set to initial gyro heading
      *   m_chassis.ResetPosition(p, h);
      *   m_chassis.EnableOdom();
      *   // m_chassis.DriveToPoint(...);
      * }
      * @endcode
      */
      void EnableOdom();
      
      void DisableOdom();

      bool IsOdomEnabled() {
        return m_odomEnabled;
      }

      /**
      * Configures inertial sensors. These are assumed to have
      * already been calibrated.
      *
      * TODO - Fix the documentation's code example
      *
      * @code{.cpp}
      * void autonomous(void) {
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
      *   vpi::DifferentialDriveChassis m_chassis{m_left, m_right};
      *   m_chassis.EnableOdom();
      *   while(1) {
      *      wait(20, vex::msec);
      *   }
      * }
      * @endcode
      */
      void AddInertialSensor(vex::inertial &s) {
        m_inertials.emplace_back(s);
      }

      /**
      * Configures GPS sensors. These are assumed to have
      * already been calibrated.
      *
      * TODO - Fix the documentation's code example
      *
      * @code{.cpp}
      * void autonomous(void) {
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
      *   vpi::DifferentialDriveChassis m_chassis{m_left, m_right};
      *   m_chassis.EnableOdom();
      *   while(1) {
      *      wait(20, vex::msec);
      *   }
      * }
      * @endcode
      */
      void AddGpsSensor(vex::gps &s) {
        VexGpsPose2dGPS v(s);
        AddGpsSensor(v);
      }

      void AddGpsSensor(VexGpsPose2dGPS &s) {
        m_gps_sensors.emplace_back(s);
      }

      void AddTrackingWheelSensors(QLength odomTrackWidth, 
                                  QLength odomWheelDiameter,
                                  AbstractRotationSensor *leftSensor,
                                  AbstractRotationSensor *rightSensor,
                                  AbstractRotationSensor *strafeSensor=NULL) {
        m_odomTrackWidth = odomTrackWidth;
        m_odomWheelDiameter = odomWheelDiameter;
        m_leftSensor = leftSensor;
        m_rightSensor = rightSensor;
        m_strafeSensor = strafeSensor;
      }

      const Pose2d& GetPose() {
        return m_odometry.GetPose();
      }

      virtual void TurnAngle(QAngle target, 
                         QAngularSpeed turnSpeed,
                         bool waitForCompletion=true) {
        m_drivetrain->TurnAngle(target, turnSpeed, waitForCompletion);
      }

      virtual void TurnToPoint(VexGpsPose2d target, 
                       QAngularSpeed turnSpeed,
                       bool waitForCompletion=true) {
        m_drivetrain->TurnToPoint(m_odometry.GetPose(), target, turnSpeed, waitForCompletion);
      }

      virtual void DriveDistance(QLength target, 
                         QSpeed movementSpeed,
                         bool waitForCompletion=true) {
        m_drivetrain->DriveDistance(target, movementSpeed, waitForCompletion);
      }

      virtual void DriveToPoint(VexGpsPose2d target, 
                        QSpeed movementSpeed,
                        bool waitForCompletion=true) {
        QAngularSpeed turnSpeed = UnitUtils::convertLinearSpeedToRotationalSpeed(movementSpeed,
                                                                                m_driveWheelDiameter,
                                                                                m_gearRatio);
        m_drivetrain->TurnToPoint(m_odometry.GetPose(), target, turnSpeed, true);
        Pose2d postTurnPose = m_odometry.GetPose();  // Turn may have changed our position a bit
        m_drivetrain->DriveDistance(postTurnPose.DistanceTo(target), movementSpeed, waitForCompletion);
      }

      virtual void DriveChassisSpeeds(ChassisSpeeds cs) {
        m_drivetrain->DriveChassisSpeeds(cs);
      }

      virtual bool IsMoving();

      virtual void SetDebug(bool b) {m_debug = b;}

      virtual void SetGpsQualityThreshold(double d) {
        GPS_SENSOR_QUALITY_THRESHOLD = d;
      }
      
      virtual void SetMultipleGPSHandler(MultipleGpsHandler g) {
        m_gpsHandler = g;
      }

      QSpeed LeftMotorSpeed() {
        return m_drivetrain->LeftMotorSpeed();
      }

      QSpeed RightMotorSpeed() {
        return m_drivetrain->RightMotorSpeed();
      }

      virtual ChassisSpeeds GetChassisSpeed() {
        return m_drivetrain->GetChassisSpeed();
      }

    protected:
      vex::gearSetting m_gearSetting;
      QLength m_driveTrackWidth;
      QLength m_driveWheelDiameter;
      double m_gearRatio;
      QLength m_odomTrackWidth;
      QLength m_odomWheelDiameter;
      DifferentialDrive *m_drivetrain = NULL;
      DifferentialDriveOdometry m_odometry;
      AbstractRotationSensor *m_leftSensor = NULL;
      AbstractRotationSensor *m_rightSensor = NULL;
      AbstractRotationSensor *m_strafeSensor = NULL;
      std::vector<vex::inertial> m_inertials;
      std::vector<VexGpsPose2dGPS> m_gps_sensors;
      double GPS_SENSOR_QUALITY_THRESHOLD = 96;
      bool m_odomEnabled = false;
      bool m_debug = false;
      MultipleGpsHandler m_gpsHandler = MultipleGpsHandler::FIRST_ABOVE_QUALITY;

      virtual const Pose2d& UpdateOdometry();
      virtual bool SetFirstQualityGPS();
      virtual bool SetHighestQualityGPS();
      virtual bool SetWeightedAverageGPS();

    private:
      vex::task *m_odomTask = NULL;
      static int _trampoline(void *p_this);

      void SafeOdomReset() {
        bool bWasOdomEnabled = IsOdomEnabled();
        if(bWasOdomEnabled) {
          DisableOdom();
        }
        UnsafeOdomReset();
        if(bWasOdomEnabled) {
          EnableOdom();
        }
      }

      void UnsafeOdomReset() {
        m_leftSensor->Reset();
        m_rightSensor->Reset();
        if(m_strafeSensor != NULL) {
          m_strafeSensor->Reset();
        }
      }
  };
} // end vpi
