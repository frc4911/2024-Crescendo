// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots;

import com.cyberknights4911.WhamControllerBinding;
import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.control.ButtonAction;
import com.cyberknights4911.control.StickAction;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.slurpp.Slurpp;
import com.cyberknights4911.util.Alliance;
import com.cyberknights4911.util.Field;
import com.cyberknights4911.util.GameAlerts;
import com.cyberknights4911.vision.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import org.littletonrobotics.junction.LoggedRobot;

public final class Wham implements RobotContainer {
  private final Drive drive;
  private final Slurpp slurpp;
  private final Vision vision;
  private final WhamControllerBinding binding;

  private boolean firstAlertHasRun = false;
  private boolean secondAlertHasRun = false;

  @Inject
  public Wham(Drive drive, Slurpp slurpp, Vision vision) {
    this.drive = drive;
    this.slurpp = slurpp;
    this.vision = vision;
    binding = new WhamControllerBinding();

    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            binding.supplierFor(StickAction.FORWARD),
            binding.supplierFor(StickAction.STRAFE),
            binding.supplierFor(StickAction.ROTATE)));

    binding.triggersFor(ButtonAction.Brake).whileTrue(drive.stopWithX());

    binding.triggersFor(ButtonAction.ZeroGyro).onTrue(zeroPoseToCurrentRotation());

    binding
        .triggersFor(ButtonAction.ZeroSpeaker)
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      new Pose2d(
                          new Translation2d(
                              Units.inchesToMeters(602.73), Units.inchesToMeters(218.42)),
                          new Rotation2d()));
                },
                drive));

    binding
        .triggersFor(ButtonAction.AmpLockOn)
        .whileTrue(
            drive.pointToAngleDrive(
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                Math.PI / 2));

    double speakerX = 0;
    double speakerY = 0;
    if (Alliance.isRed()) {
      speakerX = 652.73;
      speakerY = 218.42;
    } else {
      // TODO: put the values for the blue speaker here
      speakerX = 0;
      speakerY = 0;
    }
    binding
        .triggersFor(ButtonAction.SpeakerLockOn)
        .whileTrue(
            drive.pointToPointDrive(
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                speakerX,
                speakerY));
  }

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {
    if (!firstAlertHasRun && GameAlerts.shouldAlert(GameAlerts.Endgame1)) {
      firstAlertHasRun = true;
      CommandScheduler.getInstance()
          .schedule(
              Commands.runOnce(() -> binding.setDriverRumble(true))
                  .withTimeout(1.5)
                  .andThen(() -> binding.setDriverRumble(false))
                  .withTimeout(1.0));
    }

    if (!secondAlertHasRun && GameAlerts.shouldAlert(GameAlerts.Endgame2)) {
      secondAlertHasRun = true;
      CommandScheduler.getInstance()
          .schedule(
              Commands.runOnce(() -> binding.setDriverRumble(true))
                  .withTimeout(1.0)
                  .andThen(() -> binding.setDriverRumble(false))
                  .withTimeout(0.5)
                  .andThen(() -> binding.setDriverRumble(true))
                  .withTimeout(1.0)
                  .andThen(() -> binding.setDriverRumble(false))
                  .withTimeout(0.5));
    }
  }

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    // Autos autos = new Autos(WhamConstants.DRIVE_CONSTANTS, drive, slurpp);
    // autos.addAllAutos(handler);
  }

  /**
   * Resets the robot's current pose rotation to be zero. Will not modify robot pose translation.
   */
  private Command zeroPoseToCurrentRotation() {
    return Commands.runOnce(
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Field.forwardAngle())),
            drive)
        .ignoringDisable(true);
  }
}
