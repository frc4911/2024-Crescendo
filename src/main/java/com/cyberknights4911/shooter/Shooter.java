// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import com.cyberknights4911.logging.LoggedTunableNumberFactory;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public class Shooter extends SubsystemBase {
  // Measured in OnShape
  // motor distance 5.558
  // pivot to lower motor 7.115
  private static final Translation2d MOUNT_POINT = new Translation2d(9.141, 10.882);
  private static final double MOUNT_TO_SHOOTER_FRONT = 10.5;

  private final LoggedTunableNumber beamThreshold;
  private final LoggedTunableNumber flyWheelKp;
  private final LoggedTunableNumber flyWheelKd;
  private final LoggedTunableNumber flyWheelKs;
  private final LoggedTunableNumber flyWheelKv;
  private final LoggedTunableNumber aimerKp;
  private final LoggedTunableNumber aimerKd;
  private final LoggedTunableNumber guideKp;
  private final LoggedTunableNumber guideKd;

  private final LoggedTunableNumber feedTime;
  private final LoggedTunableNumber aimTime;
  private final LoggedTunableNumber guideOutput;
  private final LoggedTunableNumber guideReverseOutput;
  private final LoggedTunableNumber fireOutput;
  private final LoggedTunableNumber aimerCollectPosition;
  private final LoggedTunableNumber aimerSubwooferPosition;
  private final LoggedTunableNumber aimerPodiumPosition;
  private final LoggedTunableNumber forwardLimit;
  private final LoggedTunableNumber backwardLimit;

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;
  private final SysIdRoutine sysId;
  private final Mechanism2d mechanism;
  private final MechanismLigament2d segment;

  @Inject
  public Shooter(
      ShooterConstants constants, ShooterIO shooterIO, LoggedTunableNumberFactory numberFactory) {
    super();
    this.shooterIO = shooterIO;
    flyWheelKs =
        numberFactory.getNumber("Shooter/flywheelkS", constants.shooterFeedForwardValues().kS());
    flyWheelKv =
        numberFactory.getNumber("Shooter/flywheelkV", constants.shooterFeedForwardValues().kV());
    flyWheelKp =
        numberFactory.getNumber("Shooter/flywheelkP", constants.shooterFeedBackValues().kP());
    flyWheelKd =
        numberFactory.getNumber("Shooter/flywheelkD", constants.shooterFeedBackValues().kD());
    aimerKp = numberFactory.getNumber("Shooter/aimerkP", constants.aimerFeedBackValues().kP());
    aimerKd = numberFactory.getNumber("Shooter/aimerkD", constants.aimerFeedBackValues().kD());
    guideKp = numberFactory.getNumber("Shooter/guidekP", constants.guideFeedBackValues().kP());
    guideKd = numberFactory.getNumber("Shooter/guidekD", constants.guideFeedBackValues().kD());
    feedTime = numberFactory.getNumber("Shooter/feedTime", constants.feedTime());
    aimTime = numberFactory.getNumber("Shooter/aimTime", constants.aimTime());
    guideOutput =
        numberFactory.getNumber("Shooter/GuideOutputPercent", constants.guidePercentOutput());
    guideReverseOutput =
        numberFactory.getNumber(
            "Shooter/GuideReverseOutputPercent", constants.guideReversePercentOutput());
    fireOutput =
        numberFactory.getNumber("Shooter/FireOutputPercent", constants.firePercentOutput());
    aimerSubwooferPosition =
        numberFactory.getNumber(
            "Shooter/AimerSubwooferPositionDegrees", constants.speakerPositionDegrees());
    aimerCollectPosition =
        numberFactory.getNumber(
            "Shooter/AimerCollectPositionDegrees", constants.collectPositionDegrees());
    aimerPodiumPosition =
        numberFactory.getNumber(
            "Shooter/AimerPodiumPositionDegrees", constants.podiumPositionDegrees());
    forwardLimit = numberFactory.getNumber("Shooter/forwardLimit", constants.aimerForwardLimit());
    backwardLimit =
        numberFactory.getNumber("Shooter/backwardLimit", constants.aimerBackwardLimit());
    beamThreshold = numberFactory.getNumber("Shooter/beamThreshold", constants.beamThreshold());

    feedforward = new SimpleMotorFeedforward(flyWheelKs.get(), flyWheelKv.get());
    shooterIO.configureShooterPID(flyWheelKp.get(), 0.0, flyWheelKd.get());
    shooterIO.configureGuidePID(guideKp.get(), 0.0, guideKd.get());
    shooterIO.configureAimerPID(aimerKp.get(), 0.0, aimerKd.get());
    shooterIO.configureLimits(forwardLimit.get(), backwardLimit.get());

    mechanism = new Mechanism2d(28.0, 28.0);
    // Where the shooter is attached to the frame
    MechanismRoot2d root = mechanism.getRoot("shooter", MOUNT_POINT.getX(), MOUNT_POINT.getY());
    // The the tilting segment. This is the portion that is moved by the aimer.
    segment = root.append(new MechanismLigament2d("segment", MOUNT_TO_SHOOTER_FRONT, 0));

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runShooterVolts(voltage.in(Volts)), null, this));
  }

  public void runShooterOutput(double percent) {
    shooterIO.setShooterOutput(percent);

    Logger.recordOutput("Shooter/FlywheelPercent", percent);
  }

  /** Run shooter open loop at the specified voltage. */
  public void runShooterVolts(double voltage) {
    shooterIO.setShooterVoltage(voltage);

    Logger.recordOutput("Shooter/FlywheelVoltage", voltage);
  }

  /** Run shooter closed loop at the specified velocity. */
  public void runShooterVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    shooterIO.setShooterVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    Logger.recordOutput("Shooter/ShooterSetpointRPM", velocityRPM);
  }

  /** Returns the current velocity in RPM. */
  public double getShooterVelocityRpm() {
    double velocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(inputs.shooterTopVelocityRadPerSec);
    Logger.recordOutput("Shooter/ShooterVelocityRPM", velocityRpm);
    return velocityRpm;
  }

  public void runAimerVolts(double voltage) {
    shooterIO.setAimerVoltage(voltage);

    Logger.recordOutput("Shooter/AimerVoltage", voltage);
  }

  /** Run aimer closed loop to the specified position. */
  public void setAimerPostion(double positionDegrees) {
    var positionRadians = Units.degreesToRadians(positionDegrees);
    shooterIO.setAimerPosition(positionRadians, 0.0);

    Logger.recordOutput("Shooter/AimerSetpointDegrees", positionDegrees);
  }

  public void runGuideOutput(double percent) {
    shooterIO.setGuideOutput(percent);

    Logger.recordOutput("Shooter/IndexerPercent", percent);
  }

  public void runGuideVolts(double voltage) {
    shooterIO.setGuideVoltage(voltage);

    Logger.recordOutput("Shooter/IndexerVoltage", voltage);
  }

  /** Run indexer closed loop at the specified velocity. */
  public void runGuideVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    shooterIO.setGuideVelocity(velocityRadPerSec);

    Logger.recordOutput("Shooter/IndexerSetpointRPM", velocityRPM);
  }

  public void stopGuide() {
    shooterIO.stopGuide();
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (flyWheelKp.hasChanged(hashCode()) || flyWheelKd.hasChanged(hashCode())) {
      shooterIO.configureShooterPID(flyWheelKp.get(), 0.0, flyWheelKd.get());
    }
    if (flyWheelKs.hasChanged(hashCode()) || flyWheelKv.hasChanged(hashCode())) {
      feedforward = new SimpleMotorFeedforward(flyWheelKs.get(), flyWheelKv.get());
    }
    if (aimerKp.hasChanged(hashCode()) || aimerKd.hasChanged(hashCode())) {
      shooterIO.configureAimerPID(aimerKp.get(), 0.0, aimerKd.get());
    }
    if (guideKp.hasChanged(hashCode()) || guideKd.hasChanged(hashCode())) {
      shooterIO.configureGuidePID(guideKp.get(), 0.0, guideKd.get());
    }
    if (forwardLimit.hasChanged(hashCode()) || backwardLimit.hasChanged(hashCode())) {
      shooterIO.configureLimits(forwardLimit.get(), backwardLimit.get());
    }

    // TODO: convert this to the actual angle (correct for gear ratio)
    segment.setAngle(Math.toDegrees(inputs.aimerPositionRad));
    // Logger.recordOutput("Shooter/Mechanism", mechanism);

    if (DriverStation.isDisabled()) {
      shooterIO.stopShooter();
      shooterIO.stopGuide();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  private void guideReverseAtTunableOutput() {
    runGuideOutput(guideReverseOutput.get());
  }

  private void runShooterAtTunableSpeed() {
    runShooterOutput(fireOutput.get());
  }

  /** Will not complete until beam break is blocked! */
  public Command collectAndWaitForNote() {
    return Commands.runOnce(
            () -> {
              shooterIO.stopShooter();
              setAimerPostion(aimerCollectPosition.get());
              runGuideOutput(guideOutput.get());
            },
            this)
        .andThen(Commands.waitUntil(() -> inputs.beamBreakValue < beamThreshold.get()))
        .andThen(backNoteUp());
  }

  /** Will not complete until beam break is unblocked! */
  private Command backNoteUp() {
    return Commands.runOnce(() -> runGuideOutput(guideReverseOutput.get()), this)
        .andThen(Commands.waitUntil(() -> inputs.beamBreakValue > beamThreshold.get()))
        .withTimeout(.5)
        .andThen(() -> stopGuide(), this);
  }

  public Command runBackwards() {
    return Commands.runOnce(() -> runGuideOutput(-guideOutput.get()), this);
  }

  public Command stow() {
    return Commands.runOnce(
        () -> {
          shooterIO.stopGuide();
          shooterIO.stopShooter();
          setAimerPostion(aimerCollectPosition.get());
        },
        this);
  }

  public Command aimSubwoofer() {
    return Commands.runOnce(() -> setAimerPostion(aimerSubwooferPosition.get()), this)
        .andThen(backNoteUp())
        .andThen(this::runShooterAtTunableSpeed, this);
  }

  public Command aimPodium() {
    return Commands.runOnce(() -> setAimerPostion(aimerPodiumPosition.get()), this)
        .andThen(backNoteUp())
        .andThen(this::runShooterAtTunableSpeed, this);
  }

  public Command fire() {
    return Commands.runOnce(() -> runGuideOutput(guideOutput.get()), this)
        .andThen(Commands.waitSeconds(feedTime.get()))
        .andThen(shooterIO::stopShooter, this)
        .andThen(shooterIO::stopGuide, this)
        .andThen(this::collectAndWaitForNote);
  }

  /**
   * Automatically and continuously sets the aiming angle for an arbitrary distance from the
   * speaker. Note: this command never finishes on its own, so don't wait for it.
   *
   * @param distanceSupplier returns the current horizontal distance from the speaker opening (not
   *     diagonal)
   */
  public Command aimContinuous(DoubleSupplier distanceSupplier) {
    return new AimContinuousCommand(this, distanceSupplier);
  }
}