// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

#include "vex.h"
#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/hal/rotation/MotorGroupRotationSensor.h"

namespace vpi {
  DifferentialDriveChassis::DifferentialDriveChassis(vex::motor_group& leftMotorGroup, 
                          vex::motor_group& rightMotorGroup, 
                          QLength driveTrackWidth, QLength driveWheelDiameter,
                          vex::gearSetting gs,
                          double gearRatio, vex::brakeType brake) : 
                  m_gearSetting(gs),
                  m_driveTrackWidth(driveTrackWidth),
                  m_driveWheelDiameter(driveWheelDiameter),
                  m_gearRatio(gearRatio),
                  m_odomTrackWidth(driveTrackWidth),
                  m_odomWheelDiameter(driveWheelDiameter),
                  m_drivetrain(driveTrackWidth, driveWheelDiameter, 
                                      leftMotorGroup, rightMotorGroup),
                  m_odometry(driveTrackWidth, driveWheelDiameter, 0 * degree, gearRatio)
                  {
    m_drivetrain.SetBrakeType(brake);
    m_drivetrain.SetGearRatio(gearRatio);
    m_leftSensor = new MotorGroupRotationSensor(leftMotorGroup, gs);
    m_rightSensor = new MotorGroupRotationSensor(rightMotorGroup, gs);
  }

  void DifferentialDriveChassis::DisableOdom() {
    if(!m_odomEnabled) {
      return;
    }
    if(m_odomTask != NULL) {
      m_odomTask->stop();
      m_odomTask = NULL;
    }
    m_odomEnabled = false;
  }

  void DifferentialDriveChassis::EnableOdom() {
    if(m_odomEnabled) {
      return;
    }
    if(m_odomTask != NULL) {
      m_odomTask->stop();
    }
    m_odomEnabled = true;
    // Reset everything before starting the odometry task
    Pose2d p = m_odometry.GetPose();
    m_leftSensor->Reset();
    m_rightSensor->Reset();
    if(m_strafeSensor != NULL) {
      m_strafeSensor->Reset();
    }
    m_odometry.ResetPosition(p);
    m_odomTask = new vex::task(DifferentialDriveChassis::_trampoline, static_cast<void *>(this));
  }

  int DifferentialDriveChassis::_trampoline(void *p_this) {
    DifferentialDriveChassis *ddc = (DifferentialDriveChassis *)p_this;
    while(1) {
      ddc->UpdateOdometry();

      this_thread::sleep_for(20);
    }

    return 0;
  }

  const Pose2d& DifferentialDriveChassis::UpdateOdometry() {
    if(m_gps_sensors.size() > 0) {
      int i = 0;
      for(auto gps : m_gps_sensors) {
        if(gps.quality() > GPS_SENSOR_QUALITY_THRESHOLD) {
          QAngle h = gps.heading(rotationUnits::deg) * degree;
          QLength x = gps.xPosition(distanceUnits::in) * inch;
          QLength y = gps.yPosition(distanceUnits::in) * inch;
          // Pose2d gpsPose(x,y,h);
          VexGpsPose2d gpsPose(x,y,h);
          m_odometry.ResetPosition(gpsPose);
          // When calling m_odometry.ResetPosition, you must also
          // reset the sensors feeding into it
          m_leftSensor->Reset();
          m_rightSensor->Reset();
          if(m_strafeSensor != NULL) {
            m_strafeSensor->Reset();
          }
          if(m_debug){
            logger.log(Logger::LogLevel::DEBUG, 
            "DifferentialDriveChassis::UpdateOdometry - using position from GPS %d",i);
          }
          return m_odometry.GetPose();
        }
        i++;
      }
    }

    QAngle la = m_leftSensor->GetValue();
    QAngle ra = m_rightSensor->GetValue();

    QLength ld = UnitUtils::convertRotationToDistance(la, m_odomWheelDiameter, m_gearRatio);
    QLength rd = UnitUtils::convertRotationToDistance(ra, m_odomWheelDiameter, m_gearRatio);

    if(m_inertials.size() == 0) {
      if(m_debug){
        logger.log(Logger::LogLevel::DEBUG, 
        "DifferentialDriveChassis::UpdateOdometry - using position from rotation sensors");
      }
      return m_odometry.Update(ld, rd);
    } else {
      double headingAccumulator = 0;
      for (auto i : m_inertials) {
        headingAccumulator = UnitUtils::constrainTo180((headingAccumulator + i.heading(rotationUnits::deg)) * degree).convert(degree);
      }
      if(m_debug){
        logger.log(Logger::LogLevel::DEBUG, 
        "DifferentialDriveChassis::UpdateOdometry - using position from rotation sensors and IMU");
      }
      return m_odometry.Update(headingAccumulator / (double)m_inertials.size() * radian, ld, rd);
    }
  }

  bool DifferentialDriveChassis::IsMoving() {
    return m_drivetrain.IsMoving();
  }

} // end vpi
