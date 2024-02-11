// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.drive.GyroIO;
import com.cyberknights4911.drive.GyroIOPigeon2;
import com.cyberknights4911.drive.ModuleIO;
import com.cyberknights4911.drive.ModuleIOSim;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.robot2024.Robot2024Constants;
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
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LoggedRobot;

public final class Wham implements RobotContainer {
  private final Drive drive;
  private final Slurpp slurpp;
  private final VisionSimple vision;
  private final WhamControllerBinding binding;
  private final Constants constants;

  public Wham() {
    constants = WhamConstants.WHAM;
    binding = new WhamControllerBinding();
    drive = createDrive();
    slurpp = createSlurpp();
    vision =
        new VisionSimple(
            WhamConstants.VISION_CONSTANTS,
            drive::addVisionMeasurement,
            WhamConstants.CAMERA_CONSTANTS);

    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            WhamConstants.CONTROL_CONSTANTS,
            binding.supplierFor(WhamSticks.FORWARD),
            binding.supplierFor(WhamSticks.STRAFE),
            binding.supplierFor(WhamSticks.ROTATE)));

    binding.triggersFor(WhamButtons.Brake).whileTrue(drive.stopWithX());

    binding.triggersFor(WhamButtons.ZeroGyro).onTrue(drive.zeroPoseToCurrentRotation());

    binding
        .triggersFor(WhamButtons.ZeroSpeaker)
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
        .triggersFor(WhamButtons.AmpLockOn)
        .whileTrue(
            drive.pointToAngleDrive(
                Robot2024Constants.CONTROL_CONSTANTS,
                binding.supplierFor(WhamSticks.FORWARD),
                binding.supplierFor(WhamSticks.STRAFE),
                Math.PI / 2));

    binding
        .triggersFor(WhamButtons.SpeakerLockOn)
        .whileTrue(
            drive.pointToPointDrive(
                Robot2024Constants.CONTROL_CONSTANTS,
                binding.supplierFor(WhamSticks.FORWARD),
                binding.supplierFor(WhamSticks.STRAFE),
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42)));
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
  public void onRobotPeriodic(LoggedRobot robot) {}

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    Autos autos = new Autos(WhamConstants.DRIVE_CONSTANTS, drive, slurpp);
    autos.addAllAutos(handler);
  }
}
