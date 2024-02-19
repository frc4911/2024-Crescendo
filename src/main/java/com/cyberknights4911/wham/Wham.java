// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.control.ButtonAction;
import com.cyberknights4911.control.StickAction;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.drive.GyroIO;
import com.cyberknights4911.drive.GyroIOPigeon2;
import com.cyberknights4911.drive.ModuleIO;
import com.cyberknights4911.drive.ModuleIOSim;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.robot2024.Robot2024Constants;
import com.cyberknights4911.util.Alliance;
import com.cyberknights4911.util.GameAlerts;
import com.cyberknights4911.vision.simple.VisionSimple;
import com.cyberknights4911.wham.drive.ModuleIOTalonFX;
import com.cyberknights4911.wham.slurpp.Slurpp;
import com.cyberknights4911.wham.slurpp.SlurppIO;
import com.cyberknights4911.wham.slurpp.SlurppIOReal;
import com.cyberknights4911.wham.slurpp.SlurppIOSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LoggedRobot;

public final class Wham implements RobotContainer {
  private final Drive drive;
  private final Slurpp slurpp;
  private final VisionSimple vision;
  private final WhamControllerBinding binding;
  private final Constants constants;

  private boolean firstAlertHasRun = false;
  private boolean secondAlertHasRun = false;

  public Wham() {
    constants = WhamConstants.WHAM;
    binding = new WhamControllerBinding();
    drive = createDrive();
    slurpp = createSlurpp();
    vision =
        new VisionSimple(
            WhamConstants.VISION_CONSTANTS,
            drive::addVisionMeasurement,
            WhamConstants.CAMERA_CONSTANTS_FRONT_RIGHT);

    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            WhamConstants.CONTROL_CONSTANTS,
            binding.supplierFor(StickAction.FORWARD),
            binding.supplierFor(StickAction.STRAFE),
            binding.supplierFor(StickAction.ROTATE)));

    binding.triggersFor(ButtonAction.Brake).whileTrue(drive.stopWithX());

    binding.triggersFor(ButtonAction.ZeroGyro).onTrue(drive.zeroPoseToCurrentRotation());

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
                Robot2024Constants.CONTROL_CONSTANTS,
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
                Robot2024Constants.CONTROL_CONSTANTS,
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                speakerX,
                speakerY));
  }

  private Drive createDrive() {
    switch (constants.mode()) {
      case REAL:
        return createRealDrive();
      case SIM:
        return createSimDrive();
      default:
        return createReplayDrive();
    }
  }

  // Real robot, instantiate hardware IO implementations
  private Drive createRealDrive() {
    return new Drive(
        constants,
        WhamConstants.DRIVE_CONSTANTS,
        new GyroIOPigeon2(WhamConstants.DRIVE_CONSTANTS),
        new ModuleIOTalonFX(
            WhamConstants.DRIVE_CONSTANTS, WhamConstants.DRIVE_CONSTANTS.frontLeft()),
        new ModuleIOTalonFX(
            WhamConstants.DRIVE_CONSTANTS, WhamConstants.DRIVE_CONSTANTS.frontRight()),
        new ModuleIOTalonFX(
            WhamConstants.DRIVE_CONSTANTS, WhamConstants.DRIVE_CONSTANTS.backLeft()),
        new ModuleIOTalonFX(
            WhamConstants.DRIVE_CONSTANTS, WhamConstants.DRIVE_CONSTANTS.backRight()));
  }

  // Sim robot, instantiate physics sim IO implementations
  private Drive createSimDrive() {
    return new Drive(
        constants,
        WhamConstants.DRIVE_CONSTANTS,
        new GyroIO() {},
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim());
  }

  // Replayed robot, disable IO implementations
  private Drive createReplayDrive() {
    return new Drive(
        constants,
        WhamConstants.DRIVE_CONSTANTS,
        new GyroIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {},
        new ModuleIO() {});
  }

  private Slurpp createSlurpp() {
    switch (constants.mode()) {
      case REAL:
        return createRealSlurpp();
      case SIM:
        return createSimSlurpp();
      default:
        return createReplaySlurpp();
    }
  }

  private Slurpp createRealSlurpp() {
    return new Slurpp(new SlurppIOReal());
  }

  private Slurpp createSimSlurpp() {
    return new Slurpp(new SlurppIOSim());
  }

  private Slurpp createReplaySlurpp() {
    return new Slurpp(new SlurppIO() {});
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
    Autos autos = new Autos(WhamConstants.DRIVE_CONSTANTS, drive, slurpp);
    autos.addAllAutos(handler);
  }
}
