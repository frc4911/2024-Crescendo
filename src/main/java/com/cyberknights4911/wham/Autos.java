// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.subsystems.drive.Drive;
import com.cyberknights4911.util.LocalADStarAK;
import com.cyberknights4911.wham.slurpp.Slurpp;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

final class Autos {

  private final Drive drive;
  private final Slurpp slurpp;

  public Autos(DriveConstants driveConstants, Drive drive, Slurpp slurpp) {
    this.drive = drive;
    this.slurpp = slurpp;

    double driveBaseRadius =
        Math.hypot(driveConstants.trackWidthX() / 2.0, driveConstants.trackWidthY() / 2.0);

    BooleanSupplier shouldFlipPath =
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configureHolonomic(
        drive::getPose,
        drive::setPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new HolonomicPathFollowerConfig(
            driveConstants.maxLinearSpeed(), driveBaseRadius, new ReplanningConfig()),
        shouldFlipPath,
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

    Command score =
        Commands.waitSeconds(1)
            .alongWith(
                Commands.runOnce(
                    () -> {
                      slurpp.setVoltage(12 * .6);
                    },
                    slurpp))
            .andThen(
                Commands.runOnce(
                    () -> {
                      slurpp.setVoltage(0);
                    },
                    slurpp));

    Command collect =
        Commands.waitSeconds(0.75)
            .alongWith(
                Commands.runOnce(
                    () -> {
                      slurpp.setVoltage(12 * -.2);
                    },
                    slurpp))
            .andThen(
                Commands.runOnce(
                    () -> {
                      slurpp.setVoltage(0);
                    },
                    slurpp));

    NamedCommands.registerCommand("Score", score);
    NamedCommands.registerCommand("Collect", collect);
    handler.addDefaultOption("Nothing", Commands.none());
    handler.addOption("Tranlate Test", new PathPlannerAuto("TranslationTest"));
    handler.addOption("Rotate Test", new PathPlannerAuto("RotationTest"));
    handler.addOption("Auto 1", new PathPlannerAuto("Auto1"));
    handler.addOption("Auto 2", new PathPlannerAuto("Auto2"));
    handler.addOption("Auto 3", new PathPlannerAuto("Auto3"));
    // Set up FF characterization routines
    // handler.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
  }
}
