package com.cyberknights4911.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Interface for a swerve subsystem. Implement this to for integrating with {@link
 * PathPlannerCommandFactory}.
 */
public interface AutoDrive {
  Pose2d getPose();

  void resetPose(Pose2d pose);

  ChassisSpeeds getRobotRelativeSpeeds();

  void getRobotRelativeSpeeds(ChassisSpeeds speeds);
}
