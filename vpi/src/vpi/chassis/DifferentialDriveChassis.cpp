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

  bool DifferentialDriveChassis::SetFirstQualityGPS() {
    int i = 0;
    for(auto gps : m_gps_sensors) {
      VexGpsPose2d gpsPose = gps.GetValue();
      if(gpsPose.Quality() > GPS_SENSOR_QUALITY_THRESHOLD) {
        m_odometry.ResetPosition(gpsPose);
        // When calling m_odometry.ResetPosition, you must also
        // reset the sensors feeding into it
        UnsafeOdomReset();
        if(m_debug){
          logger.log(Logger::LogLevel::DEBUG, 
          "DifferentialDriveChassis::GetFirstQualityGPS - using position from GPS %d",i);
        }
        return true;
      }
      i++;
    }
    return false;
  }

  bool DifferentialDriveChassis::SetHighestQualityGPS() {
    int i = 0;
    int j = 0;
    double cur_quality = -99;
    VexGpsPose2d bestGpsPose(-99 * meter, -99 * meter, 0_deg, 0, 0_ms);
    for(auto gps : m_gps_sensors) {
      VexGpsPose2d gpsPose = gps.GetValue();
      double q = gpsPose.Quality();
      if(q >= GPS_SENSOR_QUALITY_THRESHOLD && q > cur_quality) {
        cur_quality = q;
        bestGpsPose = VexGpsPose2d(gpsPose.X(), gpsPose.Y(), gpsPose.Theta(), gpsPose.Quality(), gpsPose.Timestamp());
        i++;
      }
      j++;
    }
    if(m_debug){
      logger.log(Logger::LogLevel::DEBUG, 
      "DifferentialDriveChassis::GetHighestQualityGPS - using position from GPS %d",j);
    }
    if(i>0) {
      m_odometry.ResetPosition(bestGpsPose);
      // When calling m_odometry.ResetPosition, you must also
      // reset the sensors feeding into it
      UnsafeOdomReset();
      return true;
    } else {
      return false;
    }
  }

  bool DifferentialDriveChassis::SetWeightedAverageGPS() {
    int i = 0;
    int j = 0;

    VexGpsPose2d bestGpsPose(-99 * meter, -99 * meter, 0_deg, 0, 0_ms);
    std::vector<VexGpsPose2d> v_poses;
    for(auto gps : m_gps_sensors) {
      VexGpsPose2d gpsPose = gps.GetValue();
      double q = gpsPose.Quality();
      if(q >= GPS_SENSOR_QUALITY_THRESHOLD) {
        v_poses.emplace_back(gpsPose);

        i++;
      }
      j++;
    }
    if(m_debug){
      logger.log(Logger::LogLevel::DEBUG, 
      "DifferentialDriveChassis::SetWeightedAverageGPS - using position from %d GPS sensor readings",i);
    }
    if(i>0) {
      VexGpsPose2d wavgPose = VexGpsPose2d::WeightedAverage(v_poses);
      m_odometry.ResetPosition(wavgPose);
      // When calling m_odometry.ResetPosition, you must also
      // reset the sensors feeding into it
      UnsafeOdomReset();
      return true;
    } else {
      return false;
    }
  }

  const Pose2d& DifferentialDriveChassis::UpdateOdometry() {
    if(m_gps_sensors.size() > 0) {
      if(m_gpsHandler == MultipleGpsHandler::FIRST_ABOVE_QUALITY) {
        if(SetFirstQualityGPS()) {
          return m_odometry.GetPose();
        }
      } else if(m_gpsHandler == MultipleGpsHandler::HIGHEST_QUALITY) {
        if(SetHighestQualityGPS()) {
          return m_odometry.GetPose();
        }
      } else if(m_gpsHandler == MultipleGpsHandler::QUALITY_WEIGHTEDAVERAGE) {
        if(SetWeightedAverageGPS()) {
          return m_odometry.GetPose();
        }
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
      bool bCloseTo180 = false;
      int j = 0;
      for (auto i : m_inertials) {
        if(j == 0) {
          if(fabs(i.heading(rotationUnits::deg)) > 135) {
            bCloseTo180 = true;
          }
        }
        if(bCloseTo180) {
          // Use 0:360 range
          headingAccumulator = headingAccumulator + UnitUtils::constrainTo360(i.heading(rotationUnits::deg) * degree).convert(degree);
        } else {
          // Use -180:180 range
          headingAccumulator = headingAccumulator + UnitUtils::constrainTo180(i.heading(rotationUnits::deg) * degree).convert(degree);
        }
        j++;
      }
      if(m_debug){
        logger.log(Logger::LogLevel::DEBUG, 
        "DifferentialDriveChassis::UpdateOdometry - using position from rotation sensors and IMU");
      }
      return m_odometry.Update(headingAccumulator / (double)m_inertials.size() * degree, ld, rd);
    }
  }

  bool DifferentialDriveChassis::IsMoving() {
    return m_drivetrain.IsMoving();
  }

} // end vpi
