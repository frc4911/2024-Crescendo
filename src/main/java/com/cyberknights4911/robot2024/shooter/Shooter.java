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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber beamThreshold =
      new LoggedTunableNumber("Shooter/beamThreshold");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");
  private static final LoggedTunableNumber feedTime = new LoggedTunableNumber("Shooter/feedTime");
  private static final LoggedTunableNumber speakerFireVelocityRPM =
      new LoggedTunableNumber("Shooter/SpeakerFireVelocityRPM");
  private static final LoggedTunableNumber speakerWarmUpVelocityRpm =
      new LoggedTunableNumber("Shooter/SpeakerWarmUpVelocityRpm");
  private static final LoggedTunableNumber shooterRpmError =
      new LoggedTunableNumber("Shooter/ErrorRPM");
  private static final LoggedTunableNumber shooterStowPosition =
      new LoggedTunableNumber("Shooter/StowPosition");
  private static final LoggedTunableNumber shooterPositionError =
      new LoggedTunableNumber("Shooter/PositionErrorDegrees");
  private static final LoggedTunableNumber indexVelocityRpm =
      new LoggedTunableNumber("Shooter/IndexVelocityRPM");
  private static final LoggedTunableNumber feedVelocityRpm =
      new LoggedTunableNumber("Shooter/FeedVelocityRPM");
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
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    feedTime.initDefault(constants.feedTime());
    speakerFireVelocityRPM.initDefault(constants.speakerFireVelocityRpm());
    speakerWarmUpVelocityRpm.initDefault(constants.speakerWarmUpVelocityRpm());
    shooterRpmError.initDefault(constants.errorVelocityRpm());
    forwardLimit.initDefault(constants.aimerForwardLimit());
    backwardLimit.initDefault(constants.aimerBackwardLimit());
    indexVelocityRpm.initDefault(constants.indexVelocityRpm());
    feedVelocityRpm.initDefault(constants.feedVelocityRpm());
    beamThreshold.initDefault(constants.beamThreshold());

    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    shooterIO.configureShooterPID(kP.get(), 0.0, kD.get());

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

  /** Run shooter open loop at the specified voltage. */
  public void runShooterVolts(double volts) {
    shooterIO.setShooterVoltage(volts);
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

  /** Run aimer closed loop to the specified position. */
  public void setAimerPostion(double positionDegrees) {
    var positionRadians = Units.degreesToRadians(positionDegrees);
    shooterIO.setAimerPosition(positionRadians, 0.0);

    Logger.recordOutput("Shooter/AimerSetpointDegrees", positionDegrees);
  }

  /** Run indexer closed loop at the specified velocity. */
  public void runIndexerVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    shooterIO.setGuideVelocity(velocityRadPerSec);

    Logger.recordOutput("Shooter/IndexerSetpointRPM", velocityRPM);
  }

  public void stopIndexer() {
    shooterIO.stopGuide();
  }

  public boolean isBeamBreakBlocked() {
    return inputs.beamBreakValue > beamThreshold.get();
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      shooterIO.configureShooterPID(kP.get(), 0.0, kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    }
    if (forwardLimit.hasChanged(hashCode()) || backwardLimit.hasChanged(hashCode())) {
      shooterIO.configureLimits(forwardLimit.get(), backwardLimit.get());
    }

    // TODO: convert this to the actual angle (correct for gear ratio)
    segment.setAngle(Math.toDegrees(inputs.aimerPositionRad));
    Logger.recordOutput("Shooter/Mechanism", mechanism);

    if (DriverStation.isDisabled()) {
      stopShooter();
      stopIndexer();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /**
   * Creates a command that spins the shooter at the desired speed. This command will not end until
   * the actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  private Command spinAtSpeed(LoggedTunableNumber desiredSpeed) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              runShooterVelocity(desiredSpeed.get());
            },
            this),
        Commands.waitUntil(
            () -> {
              return Math.abs(
                      Units.radiansPerSecondToRotationsPerMinute(inputs.shooterTopVelocityRadPerSec)
                          - desiredSpeed.get())
                  < shooterRpmError.get();
            }));
  }

  /**
   * Creates a command that spins the shooter at the fast speed. This command will not end until the
   * actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  public Command firingSpeed() {
    return spinAtSpeed(speakerFireVelocityRPM);
  }

  /**
   * Creates a command that spins the shooter at the medium speed. This command will not end until
   * the actual shooter RPM falls within a configurable margin of error. When this command ends, the
   * shooter will stay spinning at the set velocity.
   */
  public Command warmUpShooter() {
    return spinAtSpeed(speakerWarmUpVelocityRpm);
  }

  /** Creates a complete command for firing a note */
  public Command fireNote() {
    // sequence:
    // 1. spin shooter at firing speed
    // 2. feed gamepiece
    // 3. wait fixed delay
    // 4. stop shooter

    return firingSpeed()
        .andThen(() -> runIndexerVelocity(feedVelocityRpm.get()))
        .andThen(Commands.waitSeconds(feedTime.get()))
        .andThen(this::stopShooter, this);
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

  /**
   * Creates a command that sets the aimer to a given angle. This command will not end until the
   * actual aimer position falls within a configurable margin of error. When this command ends, the
   * aimer will maintain the set angle.
   */
  public Command setAimerAngle(LoggedTunableNumber angleDegrees) {
    return Commands.runOnce(() -> setAimerPostion(angleDegrees.get()), this)
        .andThen(
            Commands.waitUntil(
                () -> {
                  return Math.abs(
                          Units.radiansToDegrees(inputs.guidePositionRad) - angleDegrees.get())
                      < shooterPositionError.get();
                }));
  }

  public Command stowShooter() {
    return setAimerAngle(shooterStowPosition);
  }

  /**
   * Creates a command for "indexing" a note. This runs the indexer at a fixed speed until the beam
   * break detects the note.
   */
  public Command indexNote() {
    return Commands.runOnce(() -> runIndexerVelocity(indexVelocityRpm.get()))
        .andThen(Commands.waitUntil(() -> isBeamBreakBlocked()))
        .andThen(() -> stopIndexer());
  }
}
