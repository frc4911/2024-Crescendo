// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimatorExperimental;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps three pose estimators for testing purposes. One pose estimator is base purely on odometry,
 * the second is based on odometry plus vision measurements. The third is also based on odometry and
 * vision, but is an experimental pose estimator.
 * See: https://github.com/wpilibsuite/allwpilib/pull/5473
 * 
 * This is probably really bad for performance, but it's good for comparison testing. Disable
 * pose estimators in this class to improve performance.
 */
public class PoseTracker {
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveDrivePoseEstimator odometryPlusVision;
  private final SwerveDrivePoseEstimatorExperimental experimental;

  public PoseTracker(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters) {
    odometry =
        new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    odometryPlusVision =
        new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    experimental =
        new SwerveDrivePoseEstimatorExperimental(
            kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    Pose2d odometryPose = odometry.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    Pose2d visionPose =
        odometryPlusVision.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    Pose2d expPose = experimental.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);

    return expPose;
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    odometryPlusVision.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    experimental.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public Pose2d getEstimatedPosition() {
    Pose2d odometryPose = odometry.getEstimatedPosition();
    Pose2d visionPose = odometryPlusVision.getEstimatedPosition();
    Pose2d expPose = experimental.getEstimatedPosition();

    Logger.recordOutput("Odometry/Basic", odometryPose);
    Logger.recordOutput("Odometry/Vision", visionPose);
    Logger.recordOutput("Odometry/Experimental", expPose);

    return expPose;
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
    odometryPlusVision.resetPosition(gyroAngle, modulePositions, poseMeters);
    experimental.resetPosition(gyroAngle, modulePositions, poseMeters);
  }
}