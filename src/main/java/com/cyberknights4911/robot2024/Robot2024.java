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
import com.cyberknights4911.drive.ModuleIOTalonFX;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.robot2024.climb.Climb;
import com.cyberknights4911.robot2024.climb.ClimbIO;
import com.cyberknights4911.robot2024.climb.ClimbIOSim;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.collect.CollectIO;
import com.cyberknights4911.robot2024.collect.CollectIOReal;
import com.cyberknights4911.robot2024.collect.CollectIOSim;
import com.cyberknights4911.robot2024.control.ControllerBinding;
import com.cyberknights4911.robot2024.indexer.Indexer;
import com.cyberknights4911.robot2024.indexer.IndexerIO;
import com.cyberknights4911.robot2024.indexer.IndexerIOReal;
import com.cyberknights4911.robot2024.shooter.Shooter;
import com.cyberknights4911.robot2024.shooter.ShooterIO;
import com.cyberknights4911.robot2024.shooter.ShooterIOReal;
import com.cyberknights4911.robot2024.shooter.ShooterIOSim;
import com.cyberknights4911.util.Field;
import com.cyberknights4911.util.GameAlerts;
import com.cyberknights4911.util.SparkBurnManager;
import com.cyberknights4911.vision.simple.VisionSimple;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LoggedRobot;

/** The main class for the 2024 robot to be named at a future date. */
public final class Robot2024 implements RobotContainer {
  private final Climb climb;
  private final Collect collect;
  private final Indexer indexer;
  private final Shooter shooter;
  private final VisionSimple vision;
  private final Drive drive;
  private final Constants constants;
  private final ControllerBinding binding;
  private final SparkBurnManager burnManager;

  public Robot2024() {
    constants = Constants.get();
    burnManager = new SparkBurnManager(constants);
    climb = createClimb();
    collect = createCollect();
    indexer = createIndexer();
    shooter = createShooter();
    drive = createDrive();
    vision = createVision();

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

    // TODO: Maybe remove this or make it harder to do by accident.
    binding.triggersFor(ButtonAction.ZeroGyro).onTrue(zeroPoseToCurrentRotation());

    binding.triggersFor(ButtonAction.Brake).whileTrue(drive.stopWithX());

    binding
        .triggersFor(ButtonAction.ZeroSpeaker)
        .onTrue(Commands.runOnce(() -> drive.setPose(Field.subWooferIndex()), drive));

    binding
        .triggersFor(ButtonAction.AmpLockOn)
        .whileTrue(
            drive.pointToAngleDrive(
                Robot2024Constants.CONTROL_CONSTANTS,
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                Field.ampOpeningAngle().getRadians()));

    Translation2d speakerOpening = Field.speakerOpening();
    // TODO: combine with auto-aim.
    binding
        .triggersFor(ButtonAction.SpeakerLockOn)
        .whileTrue(
            drive.pointToPointDrive(
                Robot2024Constants.CONTROL_CONSTANTS,
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                Units.inchesToMeters(speakerOpening.getX()),
                Units.inchesToMeters(speakerOpening.getY())));

    binding.triggersFor(ButtonAction.StowCollector).onTrue(stowEverything());

    binding.triggersFor(ButtonAction.CollectNote).onTrue(collectNote());

    binding.triggersFor(ButtonAction.AimSubwoofer).onTrue(shooter.aimSubwoofer());

    binding.triggersFor(ButtonAction.AimPodium).onTrue(shooter.aimPodium());

    binding.triggersFor(ButtonAction.FireNoteSpeaker).onTrue(shooter.fire());
    binding
        .triggersFor(ButtonAction.ReverseCollect)
        .onTrue(reverseNote())
        .onFalse(stowEverything());
  }

  private Command collectNote() {
    return collect
        .extendCollecter()
        .andThen(indexer.runIndexAtTunableOutput())
        .andThen(shooter.collectAndWaitForNote())
        .andThen(rumbleQuickTwice())
        .andThen(stowEverything());
  }

  private Command reverseNote() {
    return collect.ejectNote().andThen(indexer.runBackwards()).andThen(shooter.runBackwards());
  }

  private Command stowEverything() {
    return shooter.stow().alongWith(indexer.stop()).alongWith(collect.retractCollecter());
  }

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {
    binding.checkControllers();

    if (GameAlerts.shouldAlert(GameAlerts.Endgame1)) {
      CommandScheduler.getInstance().schedule(rumbleLongOnce());
    }

    if (GameAlerts.shouldAlert(GameAlerts.Endgame2)) {
      CommandScheduler.getInstance().schedule(rumbleMediumTwice());
    }
  }

  private Command scoreForAuto() {
    return shooter
        .aimSubwoofer()
        .andThen(Commands.waitSeconds(.2))
        .andThen(shooter.fire())
        .andThen(Commands.waitSeconds(.2));
  }

  private Command quickscoreForAuto() {
    return shooter.fire().andThen(Commands.waitSeconds(.1));
  }

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    NamedCommands.registerCommand("SHOOT_SUB", scoreForAuto());
    NamedCommands.registerCommand("COLLECT", collectNote());
    NamedCommands.registerCommand("QUICK_SHOOT", quickscoreForAuto());
    NamedCommands.registerCommand("AIM_SUB", shooter.aimSubwoofer());

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
        return new Collect(
            Robot2024Constants.COLLECT_CONSTANTS,
            new CollectIOReal(Robot2024Constants.COLLECT_CONSTANTS, burnManager));
      case REPLAY:
      default:
        return new Collect(Robot2024Constants.COLLECT_CONSTANTS, new CollectIO() {});
    }
  }

  private Indexer createIndexer() {
    switch (constants.mode()) {
      case SIM:
        return new Indexer(SimRobot2024Constants.INDEXER_CONSTANTS, new IndexerIO() {});
      case REAL:
        return new Indexer(
            Robot2024Constants.INDEXER_CONSTANTS,
            new IndexerIOReal(Robot2024Constants.INDEXER_CONSTANTS, burnManager));
      case REPLAY:
      default:
        return new Indexer(Robot2024Constants.INDEXER_CONSTANTS, new IndexerIO() {});
    }
  }

  private Shooter createShooter() {
    switch (constants.mode()) {
      case SIM:
        return new Shooter(
            SimRobot2024Constants.SHOOTER_CONSTANTS,
            new ShooterIOSim(SimRobot2024Constants.SHOOTER_CONSTANTS));
      case REAL:
        return new Shooter(
            Robot2024Constants.SHOOTER_CONSTANTS,
            new ShooterIOReal(Robot2024Constants.SHOOTER_CONSTANTS, burnManager));
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
            new ModuleIOTalonFX(
                Robot2024Constants.DRIVE_CONSTANTS, Robot2024Constants.DRIVE_CONSTANTS.frontLeft()),
            new ModuleIOTalonFX(
                Robot2024Constants.DRIVE_CONSTANTS,
                Robot2024Constants.DRIVE_CONSTANTS.frontRight()),
            new ModuleIOTalonFX(
                Robot2024Constants.DRIVE_CONSTANTS, Robot2024Constants.DRIVE_CONSTANTS.backLeft()),
            new ModuleIOTalonFX(
                Robot2024Constants.DRIVE_CONSTANTS,
                Robot2024Constants.DRIVE_CONSTANTS.backRight()));
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

  private VisionSimple createVision() {
    return new VisionSimple(
        Robot2024Constants.VISION_CONSTANTS,
        drive::addVisionMeasurement,
        Robot2024Constants.CAMERA_CONSTANTS_FRONT_LEFT,
        Robot2024Constants.CAMERA_CONSTANTS_FRONT_RIGHT);
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

  private Command rumbleLongOnce() {
    return Commands.runOnce(() -> binding.setDriverRumble(true))
        .withTimeout(1.5)
        .andThen(() -> binding.setDriverRumble(false))
        .withTimeout(1.0);
  }

  private Command rumbleMediumTwice() {
    return Commands.runOnce(() -> binding.setDriverRumble(true))
        .withTimeout(1.0)
        .andThen(() -> binding.setDriverRumble(false))
        .withTimeout(0.5)
        .andThen(() -> binding.setDriverRumble(true))
        .withTimeout(1.0)
        .andThen(() -> binding.setDriverRumble(false))
        .withTimeout(0.5);
  }

  private Command rumbleQuickTwice() {
    return Commands.runOnce(() -> binding.setDriverRumble(true))
        .withTimeout(0.5)
        .andThen(() -> binding.setDriverRumble(false))
        .withTimeout(0.25)
        .andThen(() -> binding.setDriverRumble(true))
        .withTimeout(0.5)
        .andThen(() -> binding.setDriverRumble(false))
        .withTimeout(0.25);
  }
}

// TODO:
// ehehehehehehehehehhehehehehehehehheehehhehehehehehehehehehehheehhehehehehehehehehehhehehehehehehehehehehehehe
// TODO:
// hows it going?
// TODO:
// pretty good, how about you?
// TODO:
// I am doing alright, bit tired though, can't wait to get vision working
