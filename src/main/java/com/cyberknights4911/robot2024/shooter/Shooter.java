// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber beamThreshold =
      new LoggedTunableNumber("Shooter/beamThreshold");
  private static final LoggedTunableNumber flyWheelKp =
      new LoggedTunableNumber("Shooter/flywheelkP");
  private static final LoggedTunableNumber flyWheelKd =
      new LoggedTunableNumber("Shooter/flywheelkD");
  private static final LoggedTunableNumber flyWheelKs =
      new LoggedTunableNumber("Shooter/flywheelkS");
  private static final LoggedTunableNumber flyWheelKv =
      new LoggedTunableNumber("Shooter/flywheelkV");
  private static final LoggedTunableNumber aimerKp = new LoggedTunableNumber("Shooter/aimerkP");
  private static final LoggedTunableNumber aimerKd = new LoggedTunableNumber("Shooter/aimerkD");
  private static final LoggedTunableNumber guideKp = new LoggedTunableNumber("Shooter/guidekP");
  private static final LoggedTunableNumber guideKd = new LoggedTunableNumber("Shooter/guidekD");

  private static final LoggedTunableNumber feedTime = new LoggedTunableNumber("Shooter/feedTime");
  private static final LoggedTunableNumber aimTime = new LoggedTunableNumber("Shooter/aimTime");
  private static final LoggedTunableNumber guideOutput =
      new LoggedTunableNumber("Shooter/GuideOutputPercent");
  private static final LoggedTunableNumber guideReverseOutput =
      new LoggedTunableNumber("Shooter/GuideReverseOutputPercent");
  private static final LoggedTunableNumber fireOutput =
      new LoggedTunableNumber("Shooter/FireOutputPercent");
  private static final LoggedTunableNumber aimerCollectPosition =
      new LoggedTunableNumber("Shooter/AimerCollectPositionDegrees");
  private static final LoggedTunableNumber aimerSpeakerPosition =
      new LoggedTunableNumber("Shooter/AimerSpeakerPositionDegrees");
  private static final LoggedTunableNumber aimerPodiumPosition =
      new LoggedTunableNumber("Shooter/AimerPodiumPositionDegrees");
  private static final LoggedTunableNumber aimerAmpPosition =
      new LoggedTunableNumber("Shooter/AimerAmpPositionDegrees");
  private static final LoggedTunableNumber forwardLimit =
      new LoggedTunableNumber("Shooter/forwardLimit");
  private static final LoggedTunableNumber backwardLimit =
      new LoggedTunableNumber("Shooter/backwardLimit");

  // Measured in OnShape
  // motor distance 5.558
  // pivot to lower motor 7.115
  private static final Translation2d MOUNT_POINT = new Translation2d(9.141, 10.882);
  private static final double MOUNT_TO_SHOOTER_FRONT = 10.5;

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;
  private final SysIdRoutine sysId;
  private final Mechanism2d mechanism;
  private final MechanismLigament2d segment;

  public Shooter(ShooterConstants constants, ShooterIO shooterIO) {
    super();
    this.shooterIO = shooterIO;
    flyWheelKs.initDefault(constants.shooterFeedForwardValues().kS());
    flyWheelKv.initDefault(constants.shooterFeedForwardValues().kV());
    flyWheelKp.initDefault(constants.shooterFeedBackValues().kP());
    flyWheelKd.initDefault(constants.shooterFeedBackValues().kD());
    aimerKp.initDefault(constants.aimerFeedBackValues().kP());
    aimerKd.initDefault(constants.aimerFeedBackValues().kD());
    guideKp.initDefault(constants.guideFeedBackValues().kP());
    guideKd.initDefault(constants.guideFeedBackValues().kD());
    feedTime.initDefault(constants.feedTime());
    aimTime.initDefault(constants.aimTime());
    guideOutput.initDefault(constants.guidePercentOutput());
    guideReverseOutput.initDefault(constants.guideReversePercentOutput());
    fireOutput.initDefault(constants.firePercentOutput());
    aimerSpeakerPosition.initDefault(constants.speakerPositionDegrees());
    aimerCollectPosition.initDefault(constants.collectPositionDegrees());
    aimerPodiumPosition.initDefault(constants.podiumPositionDegrees());
    aimerAmpPosition.initDefault(constants.ampPositionDegrees());
    forwardLimit.initDefault(constants.aimerForwardLimit());
    backwardLimit.initDefault(constants.aimerBackwardLimit());
    beamThreshold.initDefault(constants.beamThreshold());

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

    Logger.recordOutput("Shooter/FlywheelPerent", percent);
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

  /** Stops the shooter. */
  public void stopShooter() {
    shooterIO.stopShooter();
  }

  public void runAimerOutput(double percent) {
    shooterIO.setAimerOutput(percent);

    Logger.recordOutput("Shooter/AimerPercent", percent);
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

  public void setAimerForSpeaker() {
    setAimerPostion(aimerSpeakerPosition.get());
  }

  public void setAimerForCollect() {
    setAimerPostion(aimerCollectPosition.get());
  }

  public void setAimerForPodium() {
    setAimerPostion(aimerPodiumPosition.get());
  }

  public void setAimerForAmp() {
    setAimerPostion(aimerAmpPosition.get());
  }

  public void stopAimer() {
    shooterIO.stopAimer();
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

  public boolean isBeamBreakBlocked() {
    return inputs.beamBreakValue < .1;
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
    Logger.recordOutput("Shooter/Mechanism", mechanism);

    if (DriverStation.isDisabled()) {
      stopShooter();
      stopGuide();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  public void guideAtTunableOutput() {
    runGuideOutput(guideOutput.get());
  }

  public void guideReverseAtTunableOutput() {
    runGuideOutput(guideReverseOutput.get());
  }

  public void runShooterAtTunableSpeed() {
    runShooterOutput(fireOutput.get());
  }

  public double feedTime() {
    return feedTime.get();
  }

  public double aimTime() {
    return aimTime.get();
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
