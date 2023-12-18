// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.control.StickAction;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.subsystems.drive.Drive;
import com.cyberknights4911.subsystems.drive.GyroIO;
import com.cyberknights4911.subsystems.drive.GyroIOPigeon2;
import com.cyberknights4911.subsystems.drive.ModuleIO;
import com.cyberknights4911.subsystems.drive.ModuleIOSim;
import com.cyberknights4911.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.LoggedRobot;

public final class Wham implements RobotContainer {
  private final Drive drive;
  private final WhamControllerBinding binding;
  private final Constants constants;

  public Wham() {
    binding = new WhamControllerBinding();
    drive = createDrive();
    constants = WhamConstants.WHAM;

    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            WhamConstants.CONTROL_CONSTANTS,
            binding.supplierFor(StickAction.FORWARD),
            binding.supplierFor(StickAction.STRAFE),
            binding.supplierFor(StickAction.ROTATE)));

    binding.triggersFor(WhamButtons.Brake).whileTrue(drive.stopWithX());

    binding.triggersFor(WhamButtons.ZeroGyro).onTrue(drive.zeroPoseToCurrentRotation());
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
        new GyroIOPigeon2(),
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

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {}

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    Autos autos = new Autos(WhamConstants.DRIVE_CONSTANTS, drive);
    autos.addAllAutos(handler);
  }
}
