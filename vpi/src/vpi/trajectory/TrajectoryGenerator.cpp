// Copyright (c) VexWPIApi contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the VexWPIApi BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "vpi/trajectory/TrajectoryGenerator.h"

#include <utility>

#include "vpi/spline/SplineHelper.h"
#include "vpi/spline/SplineParameterizer.h"
#include "vpi/trajectory/TrajectoryParameterizer.h"

using namespace vpi;

const Trajectory TrajectoryGenerator::kDoNothingTrajectory(
    std::vector<Trajectory::State>{Trajectory::State()});
std::function<void(const char*)> TrajectoryGenerator::s_errorFunc;

Trajectory TrajectoryGenerator::GenerateTrajectory(
    Spline<3>::ControlVector initial,
    const std::vector<Translation2d>& interiorWaypoints,
    Spline<3>::ControlVector end, const TrajectoryConfig& config) {
  const Transform2d flip{Translation2d(), Rotation2d(180_deg)};
  // Make theta normal for trajectory generation if path is reversed.
  // Flip the headings.
  if (config.IsReversed()) {
    initial.x[1] *= -1;
    initial.y[1] *= -1;
    end.x[1] *= -1;
    end.y[1] *= -1;
  }

  std::vector<vpi::Pose2dWithCurvature> points;
  points =
      SplinePointsFromSplines(SplineHelper::CubicSplinesFromControlVectors(
          initial, interiorWaypoints, end));

  // After trajectory generation, flip theta back so it's relative to the
  // field. Also fix curvature.
  if (config.IsReversed()) {
    for (auto& point : points) {
      point = {point.pose + flip, -point.curvature};
    }
  }

  return TrajectoryParameterizer::TimeParameterizeTrajectory(
      points, config.Constraints(), config.StartVelocity(),
      config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
      config.IsReversed());
}

Trajectory TrajectoryGenerator::GenerateTrajectory(
    const VexGpsPose2d& start, const std::vector<Translation2d>& interiorWaypoints,
    const VexGpsPose2d& end, const TrajectoryConfig& config) {
  std::array<Spline<3>::ControlVector, 2> cv = SplineHelper::CubicControlVectorsFromWaypoints(
      start, interiorWaypoints, end);
  return GenerateTrajectory(cv[0], interiorWaypoints, cv[1], config);
}

Trajectory TrajectoryGenerator::GenerateTrajectory(
    std::vector<Spline<5>::ControlVector> controlVectors,
    const TrajectoryConfig& config) {
  const Transform2d flip{Translation2d(), Rotation2d(180_deg)};
  // Make theta normal for trajectory generation if path is reversed.
  if (config.IsReversed()) {
    for (auto& vector : controlVectors) {
      // Flip the headings.
      vector.x[1] *= -1;
      vector.y[1] *= -1;
    }
  }

  std::vector<vpi::Pose2dWithCurvature> points;
  points = SplinePointsFromSplines(
      SplineHelper::QuinticSplinesFromControlVectors(controlVectors));

  // After trajectory generation, flip theta back so it's relative to the
  // field. Also fix curvature.
  if (config.IsReversed()) {
    for (auto& point : points) {
      point = {point.pose + flip, -point.curvature};
    }
  }

  return TrajectoryParameterizer::TimeParameterizeTrajectory(
      points, config.Constraints(), config.StartVelocity(),
      config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
      config.IsReversed());
}

Trajectory TrajectoryGenerator::GenerateTrajectory(
    const std::vector<Pose2d>& waypoints, const TrajectoryConfig& config) {
  auto newWaypoints = waypoints;
  const Transform2d flip{Translation2d(), Rotation2d(180_deg)};
  if (config.IsReversed()) {
    for (auto& waypoint : newWaypoints) {
      waypoint = waypoint + flip;
    }
  }

  std::vector<Pose2dWithCurvature> points;
  points = SplinePointsFromSplines(
      SplineHelper::QuinticSplinesFromWaypoints(newWaypoints));

  // After trajectory generation, flip theta back so it's relative to the
  // field. Also fix curvature.
  if (config.IsReversed()) {
    for (auto& point : points) {
      point = {point.pose + flip, -point.curvature};
    }
  }

  return TrajectoryParameterizer::TimeParameterizeTrajectory(
      points, config.Constraints(), config.StartVelocity(),
      config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(),
      config.IsReversed());
}