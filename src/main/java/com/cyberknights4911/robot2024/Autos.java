// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.robot2024.climb.Climb;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.shooter.Shooter;
import com.cyberknights4911.util.Alliance;
import com.cyberknights4911.util.LocalADStarAK;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public final class Autos {
  private final Climb climb;
  private final Collect collect;
  private final Shooter shooter;
  private final Drive drive;

  public Autos(
      DriveConstants driveConstants, Climb climb, Collect collect, Shooter shooter, Drive drive) {
    this.climb = climb;
    this.collect = collect;
    this.shooter = shooter;
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
        Alliance::isRed,
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

  public void addAllAutos(AutoCommandHandler handler) {
    handler.addDefaultOption("Nothing", Commands.none());
    handler.addOption("Leave, Bro", new PathPlannerAuto("LEAVE"));
    handler.addOption("Score+Leave Source", new PathPlannerAuto("SHOOT_AND_LEAVE_SOURCE"));
    handler.addOption("Score+Wait+Leave Amp", new PathPlannerAuto("SHOOT_WAIT_AND_LEAVE_AMP"));
    handler.addOption("Two Piece Source", new PathPlannerAuto("SHOOT_AND_LEAVE_SOURCE_COLLECT"));
    // handler.addOption("Hopez And Dreamz", new PathPlannerAuto("HopesAndDreams"));

    // handler.addOption("Translate Test", new PathPlannerAuto("TranslationTest"));

    // addCharacterization("Climb", climb.getSysId(), handler);
    // addCharacterization("Collector", collect.getSysId(), handler);
    // addCharacterization("Shooter", shooter.getSysId(), handler);
    // addCharacterization("Drive", drive.getSysId(), handler);
  }

  private void addCharacterization(String name, SysIdRoutine routine, AutoCommandHandler handler) {
    handler.addOption(
        name + " SysId (Quasistatic Forward)",
        routine.quasistatic(SysIdRoutine.Direction.kForward));
    handler.addOption(
        name + " SysId (Quasistatic Reverse)",
        routine.quasistatic(SysIdRoutine.Direction.kReverse));
    handler.addOption(
        name + " SysId (Dynamic Forward)", routine.dynamic(SysIdRoutine.Direction.kForward));
    handler.addOption(
        name + " SysId (Dynamic Reverse)", routine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
