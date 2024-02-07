// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Utility class for autonomous driving operations. */
public class AutoDriveUtils {
  // define DirectDrivePath class

  /** Represents a direct drive path with an angle and distance. */
  public static class DirectDrivePath {
    public Rotation2d angle;
    public double distance;

    /**
     * Constructs a DirectDrivePath object with the specified angle and distance.
     *
     * @param Rotation2d angle the angle of the path
     * @param distance the distance of the path
     */
    public DirectDrivePath(Rotation2d angle, double distance) {
      this.angle = angle;
      this.distance = distance;
    }
  }

  /**
   * Calculates the direct drive path based on the current robot pose.
   *
   * @param currentPose the current pose of the robot
   * @param targetPose the target pose of the robot
   * @return the DirectDrivePath object representing the calculated path
   */
  public static DirectDrivePath getDirectPath(Pose2d currentPose, Pose2d targetPose) {
    // Calculate the angle and distance to move the robot
    Rotation2d currentAngle = currentPose.getRotation();

    // Calculate the angle to the target
    Rotation2d targetAngle = targetPose.getRotation();
    Rotation2d angle = targetAngle.minus(currentAngle);

    // Calculate the distance to the target
    double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());

    // Create a DirectDrivePath object with the calculated angle and distance
    DirectDrivePath directDrivePath = new DirectDrivePath(angle, distance);

    return directDrivePath;
  }

  /**
   * Returns the angle from the given DirectDrivePath.
   *
   * @param directDrivePath the DirectDrivePath object
   * @return the angle from the DirectDrivePath
   */
  public static Rotation2d getAngleFromDirectPath(DirectDrivePath directDrivePath) {
    return directDrivePath.angle;
  }
}
