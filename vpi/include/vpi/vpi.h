// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Previously in vex.h
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;
extern brain Brain;
// End vex.h

//

// Per jpearman for LVGL support from:
// https://www.vexforum.com/t/setting-pictures-as-covers-for-programs/94777/11
#include "v5lvgl.h"
#include "lv_conf.h"

#undef __ARM_NEON__
#undef __ARM_NEON
#define  EIGEN_MAX_STATIC_ALIGN_BYTES 0
#include "Eigen/Dense"

#include "vpi/chassis/DifferentialDriveChassis.h"
#include "vpi/chassis/DifferentialDriveChassisPID.h"
#include "vpi/controller/SimpleWaypointFollower.h"
#include "vpi/controller/SimpleTrajectoryFollower.h"
#include "vpi/controller/RamseteController.h"
#include "vpi/controller/RamseteTrajectoryFollower.h"
#include "vpi/drive/DifferentialDrive.h"
#include "vpi/drive/MecanumDrive.h"
#include "vpi/drive/RobotDriveBase.h"
#include "vpi/filter/AngularSlewRateLimiter.h"
#include "vpi/filter/LinearSlewRateLimiter.h"
#include "vpi/geometry/Point2d.h"
#include "vpi/geometry/Pose2d.h"
#include "vpi/geometry/Rotation2d.h"
#include "vpi/geometry/Transform2d.h"
#include "vpi/geometry/Translation2d.h"
#include "vpi/geometry/Twist2d.h"
#include "vpi/geometry/Vector2d.h"
#include "vpi/geometry/VexGpsPose2d.h"
#include "vpi/hal/rotation/AbstractRotationSensor.h"
#include "vpi/hal/rotation/EncoderRotationSensor.h"
#include "vpi/hal/rotation/MotorRotationSensor.h"
#include "vpi/hal/rotation/MotorGroupRotationSensor.h"
#include "vpi/hal/rotation/PotentiometerRotationSensor.h"
#include "vpi/hal/rotation/V5RotationSensor.h"
#include "vpi/hal/motor/AngularSlewRateLimitedMotor.h"
#include "vpi/hal/motor/LinearSlewRateLimitedMotor.h"
#include "vpi/kinematics/ChassisSpeeds.h"
#include "vpi/kinematics/DifferentialDriveKinematics.h"
#include "vpi/kinematics/DifferentialDriveWheelSpeeds.h"
#include "vpi/kinematics/DifferentialDriveOdometry.h"
#include "vpi/kinematics/MecanumDriveKinematics.h"
#include "vpi/kinematics/MecanumDriveOdometry.h"
#include "vpi/kinematics/MecanumDriveWheelSpeeds.h"
#include "vpi/pid/PIDController.h"
#include "vpi/spline/CubicHermiteSpline.h"
#include "vpi/spline/QuinticHermiteSpline.h"
#include "vpi/spline/Spline.h"
#include "vpi/spline/SplineHelper.h"
#include "vpi/spline/SplineParameterizer.h"
#include "vpi/trajectory/CentripetalAccelerationConstraint.h"
#include "vpi/trajectory/DifferentialDriveKinematicsConstraint.h"
#include "vpi/trajectory/MecanumDriveKinematicsConstraint.h"
#include "vpi/trajectory/MaxVelocityConstraint.h"
#include "vpi/trajectory/Trajectory.h"
#include "vpi/trajectory/TrajectoryConfig.h"
#include "vpi/trajectory/TrajectoryConstraint.h"
#include "vpi/trajectory/TrajectoryGenerator.h"
#include "vpi/trajectory/TrajectoryParameterizer.h"
#include "vpi/trajectory/TrapezoidProfile.h"
#include "vpi/units/QAcceleration.h"
#include "vpi/units/QAngle.h"
#include "vpi/units/QAngularAcceleration.h"
#include "vpi/units/QAngularJerk.h"
#include "vpi/units/QAngularSpeed.h"
#include "vpi/units/QCharge.h"
#include "vpi/units/QCurvature.h"
#include "vpi/units/QElectricPotential.h"
#include "vpi/units/QElectricPotentialPerSpeed.h"
#include "vpi/units/QElectricPotentialPerAcceleration.h"
#include "vpi/units/QFrequency.h"
#include "vpi/units/QJerk.h"
#include "vpi/units/QLength.h"
#include "vpi/units/QSpeed.h"
#include "vpi/units/QTime.h"
#include "vpi/units/RQuantity.h"
#include "vpi/units/UnitUtils.h"
#include "vpi/utils.h"