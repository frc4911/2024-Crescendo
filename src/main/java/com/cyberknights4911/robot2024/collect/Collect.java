// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Collect extends SubsystemBase {
  private static final LoggedTunableNumber beamThreshold =
      new LoggedTunableNumber("Collect/beamThreshold");
  private static final LoggedTunableNumber ejectSpeed =
      new LoggedTunableNumber("Collect/EjectVelocityRPM");
  private static final LoggedTunableNumber collectSpeed =
      new LoggedTunableNumber("Collect/CollectVelocityRPM");
  private static final LoggedTunableNumber feedShooterSpeed =
      new LoggedTunableNumber("Collect/FeedShooterVelocityRPM");
  private static final LoggedTunableNumber collectKp = new LoggedTunableNumber("Collect/CollectKp");
  private static final LoggedTunableNumber collectKd = new LoggedTunableNumber("Collect/CollectKd");
  private static final LoggedTunableNumber ejectTime = new LoggedTunableNumber("Collect/EjectTime");
  private static final LoggedTunableNumber guideKp = new LoggedTunableNumber("Collect/GuideKp");
  private static final LoggedTunableNumber guideKd = new LoggedTunableNumber("Collect/GuideKd");

  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public Collect(CollectConstants constants, CollectIO collectIO) {
    super();
    this.collectIO = collectIO;
    collectKp.initDefault(constants.collectFeedBackValues().kP());
    collectKd.initDefault(constants.collectFeedBackValues().kD());
    feedShooterSpeed.initDefault(constants.feedShooterSpeed());
    collectSpeed.initDefault(constants.collectSpeed());
    ejectSpeed.initDefault(constants.ejectSpeed());
    ejectTime.initDefault(constants.ejectTime());
    guideKp.initDefault(constants.guideFeedBackValues().kP());
    guideKd.initDefault(constants.guideFeedBackValues().kP());
    collectIO.configureCollectPID(collectKp.get(), 0.0, collectKd.get());
    collectIO.configureGuidePID(guideKp.get(), 0.0, guideKd.get());

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Collect/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCollectVolts(voltage.in(Volts)), null, this));
  }

  public boolean isBeamBreakBlocked() {
    return inputs.beamBreakVoltage > beamThreshold.get();
  }

  /** Run collector open loop at the specified voltage. */
  public void runCollectVolts(double volts) {
    collectIO.setCollectVoltage(volts);
  }

  /** Run collector closed loop at the specified velocity. */
  public void runCollectVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    collectIO.setCollectVelocity(velocityRadPerSec);

    Logger.recordOutput("Collect/SetpointRPM", velocityRpm);
  }

  /** Run guide closed loop at the specified velocity. */
  public void runGuideVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    collectIO.setGuideVelocity(velocityRadPerSec);

    Logger.recordOutput("Collect/GuideSetpointRPM", velocityRpm);
  }

  public void stopGuide() {
    collectIO.stopGuide();
  }

  @Override
  public void periodic() {
    collectIO.updateInputs(inputs);
    Logger.processInputs("Collect", inputs);

    if (collectKp.hasChanged(hashCode()) || collectKd.hasChanged(hashCode())) {
      collectIO.configureCollectPID(collectKp.get(), 0.0, collectKd.get());
    }

    if (guideKp.hasChanged(hashCode()) || guideKd.hasChanged(hashCode())) {
      collectIO.configureGuidePID(guideKp.get(), 0.0, guideKd.get());
    }

    if (DriverStation.isDisabled()) {
      stopCollector();
      stopGuide();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /** Returns the current velocity in RPM. */
  public double getCollectVelocityRpm() {
    double velocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(inputs.collectVelocityRadPerSec);
    Logger.recordOutput("Collect/VelocityRPM", velocityRpm);
    return velocityRpm;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    collectIO.setCollectVoltage(volts);
  }

  /** Returns the average velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.collectVelocityRadPerSec;
  }

  /** Stops the collector. */
  public void stopCollector() {
    collectIO.stopCollector();
  }

  /**
   * Creates a command that runs the collector at a desired speed. The collector will stay running
   * after this command ends
   */
  private Command collectAtSpeed(LoggedTunableNumber desiredSpeed) {
    return Commands.runOnce(
        () -> {
          runCollectVelocity(desiredSpeed.get());
        },
        this);
  }

  /**
   * Creates a command that runs the collector at the proper collector speed, but stops when the
   * gamepiece reaches the sensor.
   */
  public Command collectGamePieceAndStop() {
    return collectAtSpeed(collectSpeed)
        .until(this::isBeamBreakBlocked)
        .andThen(this::stopCollector, this);
  }

  /** Creates a command that runs the collector in the mode for handing gamepieces to the shooter */
  public Command handGamePieceToShooter() {
    return collectAtSpeed(feedShooterSpeed);
  }

  /** Creates a command for ejecting gamepieces backwards, out of the collector. */
  public Command ejectGamePiece() {
    return collectAtSpeed(ejectSpeed)
        .andThen(Commands.waitSeconds(ejectTime.get()))
        .andThen(this::stopCollector, this);
  }
}
