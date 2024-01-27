// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;

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
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    collectIO.configurePID(kP.get(), 0.0, kD.get());
  }

  public boolean isBeamBreakBlocked() {
    return inputs.beamBreakVoltage > beamThreshold.get();
  }

  public void setCollectVoltage(double volts) {
    collectIO.setVoltage(volts);
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

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    collectIO.setVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    Logger.recordOutput("Collect/SetpointRPM", velocityRpm);
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRpm() {
    double velocityRpm = Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    Logger.recordOutput("Collect/VelocityRPM", velocityRpm);
    return velocityRpm;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    collectIO.setVoltage(volts);
  }

  /** Returns the average velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  /** Stops the collector. */
  public void stop() {
    collectIO.stop();
  }

  /**
   * Creates a command that runs the collector at a desired speed. The collector will stay running
   * after this command ends
   */
  private Command collectAtSpeed(LoggedTunableNumber desiredSpeed) {
    return Commands.runOnce(
        () -> {
          runVelocity(desiredSpeed.get());
        },
        this);
  }

  /**
   * Creates a command that runs the collector at the proper collector speed, but stops when the
   * gamepiece reaches the sensor.
   */
  public Command collectGamePiece() {
    return Commands.sequence(
      Commands.none(), // replace this one with a command that sets the collector speed
      Commands.none(), // replace this one with a command that waits for the sensor to trip
      Commands.none()  // replace this one with a command that stops the collector
    );
  }

  /**
   * Creates a command that runs the collector in the mode for feeding gamepieces to the shooter
   */
  public Command feedGamePieceToShooter() {
    return Commands.none(); // replace this with a command that sets the collector at the feed speed
  }

  /**
   * Creates a command for ejecting gamepieces backwards, out of the collector.
   */
  public Command ejectGamePiece() {
    return Commands.sequence(
      Commands.none(), // replace this one with a command that runs the collector backwards
      Commands.none(), // replace this one with a command that waits at a fixed interval
      Commands.none()  // replace this one with a command that stops the collector
    );
  }
}
