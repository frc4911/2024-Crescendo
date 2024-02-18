// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimatorExperimental;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps three pose estimators for testing purposes. One pose estimator is base purely on odometry,
 * the second is based on odometry plus vision measurements. The third is also based on odometry and
 * vision, but is an experimental pose estimator. See:
 * https://github.com/wpilibsuite/allwpilib/pull/5473
 *
 * <p>This is probably really bad for performance, but it's good for comparison testing. Disable
 * pose estimators in this class to improve performance.
 */
public class PoseTracker {
  private static final boolean ENABLE_BASE_ESTIMATOR = true;

  // Use for demo purposes only. This should be disabled in real usage
  private final Optional<SwerveDrivePoseEstimatorExperimental> odometry;
  private final SwerveDrivePoseEstimatorExperimental experimental;

  public PoseTracker(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters) {
    if (ENABLE_BASE_ESTIMATOR) {
      odometry =
          Optional.of(
              new SwerveDrivePoseEstimatorExperimental(
                  kinematics, gyroAngle, modulePositions, initialPoseMeters));
    } else {
      odometry = Optional.empty();
    }
    experimental =
        new SwerveDrivePoseEstimatorExperimental(
            kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    odometry.ifPresent(estimator -> estimator.update(gyroAngle, modulePositions));
    return experimental.update(gyroAngle, modulePositions);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    odometry.ifPresent(
        estimator -> estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions));
    return experimental.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    experimental.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    Logger.recordOutput("Odometry/VisionUpdate", visionRobotPoseMeters);
    experimental.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public Pose2d getEstimatedPosition() {
    odometry.ifPresent(
        estimator -> {
          Pose2d odometryPose = estimator.getEstimatedPosition();
          Logger.recordOutput("Odometry/Basic", odometryPose);
        });

    Pose2d expPose = experimental.getEstimatedPosition();
    Logger.recordOutput("Odometry/Experimental", expPose);
    return expPose;
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    odometry.ifPresent(
        estimator -> estimator.resetPosition(gyroAngle, modulePositions, poseMeters));
    experimental.resetPosition(gyroAngle, modulePositions, poseMeters);
  }
}
