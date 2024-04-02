// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.control.ButtonAction;
import com.cyberknights4911.control.ControllerBinding;
import com.cyberknights4911.control.StickAction;
import com.cyberknights4911.control.Triggers;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.slurpp.Slurpp;
import com.cyberknights4911.util.Field;
import com.cyberknights4911.util.GameAlerts;
import com.cyberknights4911.vision.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import java.util.function.DoubleSupplier;
import javax.inject.Inject;
import org.littletonrobotics.junction.LoggedRobot;

public final class Wham implements RobotContainer {
  private final Drive drive;
  private final Slurpp slurpp;
  private final Vision vision;
  private final Alliance alliance;
  private final ControllerBinding binding;
  private final Field field;
  private final Map<ButtonAction, Triggers> buttonTriggers;
  private final Map<StickAction, DoubleSupplier> stickSuppliers;

  private boolean firstAlertHasRun = false;
  private boolean secondAlertHasRun = false;

  @Inject
  public Wham(
      Drive drive,
      Slurpp slurpp,
      Vision vision,
      Alliance alliance,
      Field field,
      ControllerBinding binding,
      Map<ButtonAction, Triggers> buttonTriggers,
      Map<StickAction, DoubleSupplier> stickSuppliers) {
    this.drive = drive;
    this.slurpp = slurpp;
    this.vision = vision;
    this.alliance = alliance;
    this.field = field;
    this.binding = binding;
    this.buttonTriggers = buttonTriggers;
    this.stickSuppliers = stickSuppliers;

    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            stickSuppliers.get(StickAction.FORWARD),
            stickSuppliers.get(StickAction.STRAFE),
            stickSuppliers.get(StickAction.ROTATE)));

    buttonTriggers.get(ButtonAction.Brake).whileTrue(drive.stopWithX());

    buttonTriggers.get(ButtonAction.ZeroGyro).onTrue(zeroPoseToCurrentRotation());
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
            () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), field.forwardAngle())),
            drive)
        .ignoringDisable(true);
  }
}
