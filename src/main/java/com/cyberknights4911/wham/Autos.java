// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.commands.FeedForwardCharacterization;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.subsystems.drive.Drive;
import com.cyberknights4911.util.LocalADStarAK;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

final class Autos {

  private final Drive drive;

  public Autos(DriveConstants driveConstants, Drive drive) {
    this.drive = drive;

    double driveBaseRadius =
        Math.hypot(driveConstants.trackWidthX() / 2.0, driveConstants.trackWidthY() / 2.0);

    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            driveConstants.maxLinearSpeed(), driveBaseRadius, new ReplanningConfig()),
        drive);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  void addAllAutos(AutoCommandHandler handler) {
    handler.addDefaultOption("Nothing", Commands.none());
    handler.addOption("Tranlate Test", new PathPlannerAuto("TranslateTest"));

    // Set up FF characterization routines
    handler.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
  }
}