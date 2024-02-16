// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Collect/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Collect/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Collect/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Collect/kV");
  private static final LoggedTunableNumber ejectTime = new LoggedTunableNumber("Collect/ejectTime");

  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;
  private final SysIdRoutine sysId;

  public Collect(CollectConstants constants, CollectIO collectIO) {
    super();
    this.collectIO = collectIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    feedShooterSpeed.initDefault(constants.feedShooterSpeed());
    collectSpeed.initDefault(constants.collectSpeed());
    ejectSpeed.initDefault(constants.ejectSpeed());
    ejectTime.initDefault(constants.ejectTime());
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    collectIO.configurePID(kP.get(), 0.0, kD.get());

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
    collectIO.setCollectVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    Logger.recordOutput("Collect/SetpointRPM", velocityRpm);
  }

  /** Run guide closed loop at the specified velocity. */
  public void runGuideVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    collectIO.setGuideVelocity(velocityRadPerSec);

    Logger.recordOutput("Collect/GuideSetpointRPM", velocityRpm);
  }

  @Override
  public void periodic() {
    collectIO.updateInputs(inputs);
    Logger.processInputs("Collect", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      collectIO.configurePID(kP.get(), 0.0, kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  /** Returns SysId routine for characterization. */
  public SysIdRoutine getSysId() {
    return sysId;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRpm() {
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
  public void stop() {
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
  public Command collectGamePiece() {
    return collectAtSpeed(collectSpeed).until(this::isBeamBreakBlocked).andThen(this::stop, this);
  }

  /** Creates a command that runs the collector in the mode for feeding gamepieces to the shooter */
  public Command feedGamePieceToShooter() {
    return collectAtSpeed(feedShooterSpeed);
  }

  /** Creates a command for ejecting gamepieces backwards, out of the collector. */
  public Command ejectGamePiece() {
    return collectAtSpeed(ejectSpeed)
        .andThen(Commands.waitSeconds(ejectTime.get()))
        .andThen(this::stop, this);
  }
}
