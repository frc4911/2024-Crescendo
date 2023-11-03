package com.cyberknights4911.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Factory for creating PathPlanner auto path. */
public final class PathPlannerCommandFactory {

  public PathPlannerCommandFactory(
      HolonomicPathFollowerConfig followerConfig, AutoDrive autoDrive, Subsystem driveSubsystem) {
    AutoBuilder.configureHolonomic(
        autoDrive::getPose,
        autoDrive::resetPose,
        autoDrive::getRobotRelativeSpeeds,
        autoDrive::setRobotRelativeSpeeds,
        followerConfig,
        driveSubsystem);
  }

  public Command createAutoCommand(String name) {
    return new PathPlannerAuto(name);
  }
}
