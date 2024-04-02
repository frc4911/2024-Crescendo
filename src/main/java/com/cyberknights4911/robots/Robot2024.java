// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots;

import com.cyberknights4911.Autos;
import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.collect.Collect;
import com.cyberknights4911.control.ButtonAction;
import com.cyberknights4911.control.ControllerBinding;
import com.cyberknights4911.control.StickAction;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.indexer.Indexer;
import com.cyberknights4911.shooter.Shooter;
import com.cyberknights4911.util.Field;
import com.cyberknights4911.util.GameAlerts;
import com.cyberknights4911.vision.Vision;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import javax.inject.Inject;
import org.littletonrobotics.junction.LoggedRobot;

/** The main class for the 2024 robot to be named at a future date. */
public final class Robot2024 implements RobotContainer {
  private final Collect collect;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Vision vision;
  private final Drive drive;
  private final ControllerBinding binding;
  private final Autos autos;

  @Inject
  public Robot2024(
      Collect collect,
      Indexer indexer,
      Shooter shooter,
      Vision vision,
      Drive drive,
      ControllerBinding binding,
      Autos autos) {
    this.collect = collect;
    this.indexer = indexer;
    this.shooter = shooter;
    this.vision = vision;
    this.drive = drive;
    this.binding = binding;
    this.autos = autos;

    configureControls();
  }

  private void configureControls() {
    drive.setDefaultCommand(
        drive.joystickDrive(
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
                binding.supplierFor(StickAction.FORWARD),
                binding.supplierFor(StickAction.STRAFE),
                Field.ampOpeningAngle().getRadians()));

    Translation2d speakerOpening = Field.speakerOpening();
    // TODO: combine with auto-aim.
    binding
        .triggersFor(ButtonAction.SpeakerLockOn)
        .whileTrue(
            drive.pointToPointDrive(
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

    autos.addAllAutos(handler);
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
