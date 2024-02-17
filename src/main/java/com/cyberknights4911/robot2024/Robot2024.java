// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

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
import com.cyberknights4911.robot2024.climb.Climb;
import com.cyberknights4911.robot2024.climb.ClimbIO;
import com.cyberknights4911.robot2024.climb.ClimbIOSim;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.collect.CollectIO;
import com.cyberknights4911.robot2024.collect.CollectIOSim;
import com.cyberknights4911.robot2024.control.ControllerBinding;
import com.cyberknights4911.robot2024.drive.ModuleIOSparkFlex;
import com.cyberknights4911.robot2024.shooter.Shooter;
import com.cyberknights4911.robot2024.shooter.ShooterIO;
import com.cyberknights4911.robot2024.shooter.ShooterIOSim;
import com.cyberknights4911.util.GameAlerts;
import com.cyberknights4911.util.SparkBurnManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LoggedRobot;

/** The main class for the 2024 robot to be named at a future date. */
public final class Robot2024 implements RobotContainer {
  private final Climb climb;
  private final Collect collect;
  private final Shooter shooter;
  private final Drive drive;
  private final Constants constants;
  private final ControllerBinding binding;
  private final SparkBurnManager burnManager;

  public Robot2024() {
    constants = Constants.get();
    burnManager = new SparkBurnManager(constants);
    climb = createClimb();
    collect = createCollect();
    shooter = createShooter();
    drive = createDrive();

    binding = new ControllerBinding(Robot2024Constants.CONTROL_CONSTANTS);
    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            Robot2024Constants.CONTROL_CONSTANTS,
            binding.supplierFor(StickAction.FORWARD),
            binding.supplierFor(StickAction.STRAFE),
            binding.supplierFor(StickAction.ROTATE)));

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
    binding
        .triggersFor(ButtonAction.SpeakerLockOn)
        .whileTrue(
            drive.pointToPointDrive(
                Robot2024Constants.CONTROL_CONSTANTS,
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                Units.inchesToMeters(652.73),
                Units.inchesToMeters(218.42)));
    
    binding.triggersFor(ButtonAction.StowShooter)
        .onTrue(Commands.none());

    binding.triggersFor(ButtonAction.CollectNote)
        .onTrue(Commands.none());

    binding.triggersFor(ButtonAction.FireNote)
        .onTrue(Commands.none());

    binding.triggersFor(ButtonAction.StowClimber)
        .onTrue(Commands.none());

    binding.triggersFor(ButtonAction.ExtendClimber)
        .onTrue(Commands.none());
  }

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {
    binding.checkControllers();

    if (GameAlerts.shouldAlert(GameAlerts.Endgame1)) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.runOnce(() -> binding.setDriverRumble(true))
                  .withTimeout(1.5)
                  .andThen(() -> binding.setDriverRumble(false))
                  .withTimeout(1.0));
    }

    if (GameAlerts.shouldAlert(GameAlerts.Endgame2)) {
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
    Autos autos = new Autos(Robot2024Constants.DRIVE_CONSTANTS, climb, collect, shooter, drive);
    autos.addAllAutos(handler);
  }

  private Climb createClimb() {
    switch (constants.mode()) {
      case SIM:
        return new Climb(
            SimRobot2024Constants.CLIMB_CONSTANTS,
            new ClimbIOSim(SimRobot2024Constants.CLIMB_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Climb(Robot2024Constants.CLIMB_CONSTANTS, new ClimbIO() {});
    }
  }

  private Collect createCollect() {
    switch (constants.mode()) {
      case SIM:
        return new Collect(
            SimRobot2024Constants.COLLECT_CONSTANTS,
            new CollectIOSim(SimRobot2024Constants.COLLECT_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Collect(Robot2024Constants.COLLECT_CONSTANTS, new CollectIO() {});
    }
  }

  private Shooter createShooter() {
    switch (constants.mode()) {
      case SIM:
        return new Shooter(
            SimRobot2024Constants.SHOOTER_CONSTANTS,
            new ShooterIOSim(SimRobot2024Constants.SHOOTER_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Shooter(Robot2024Constants.SHOOTER_CONSTANTS, new ShooterIO() {});
    }
  }

  private Drive createDrive() {
    switch (constants.mode()) {
      case SIM:
        return new Drive(
            constants,
            Robot2024Constants.DRIVE_CONSTANTS,
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
      case REAL:
        return new Drive(
            constants,
            Robot2024Constants.DRIVE_CONSTANTS,
            new GyroIOPigeon2(Robot2024Constants.DRIVE_CONSTANTS),
            new ModuleIOSparkFlex(
                Robot2024Constants.DRIVE_CONSTANTS,
                Robot2024Constants.DRIVE_CONSTANTS.frontLeft(),
                burnManager),
            new ModuleIOSparkFlex(
                Robot2024Constants.DRIVE_CONSTANTS,
                Robot2024Constants.DRIVE_CONSTANTS.frontRight(),
                burnManager),
            new ModuleIOSparkFlex(
                Robot2024Constants.DRIVE_CONSTANTS,
                Robot2024Constants.DRIVE_CONSTANTS.backLeft(),
                burnManager),
            new ModuleIOSparkFlex(
                Robot2024Constants.DRIVE_CONSTANTS,
                Robot2024Constants.DRIVE_CONSTANTS.backRight(),
                burnManager));
      case REPLAY:
      default:
        return new Drive(
            constants,
            Robot2024Constants.DRIVE_CONSTANTS,
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }
}

// TODO:
// ehehehehehehehehehhehehehehehehehheehehhehehehehehehehehehehheehhehehehehehehehehehhehehehehehehehehehehehehe
