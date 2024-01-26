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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Collect extends SubsystemBase {
  private static final LoggedTunableNumber collectMediumSpeed =
      new LoggedTunableNumber("Collect/IntakeMediumVelocityRPM", 1_000);
  private static final LoggedTunableNumber collectSlowSpeed =
      new LoggedTunableNumber("Collect/IntakeSlowVelocityRPM", 500);
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
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    collectIO.configurePID(kP.get(), 0.0, kD.get());
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
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    collectIO.setVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    // Log collector setpoint
    Logger.recordOutput("Collect/SetpointRPM", velocityRPM);
  }

  /** Stops the collector. */
  public void stop() {
    collectIO.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    collectIO.setVoltage(volts);
  }

  /** Returns the average velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  /** Creates a command that collects at the medium speed stops when interrupted. */
  public Command collectMedium() {
    return Commands.startEnd(
        () -> {
          runVelocity(collectMediumSpeed.get());
        },
        () -> {
          stop();
        },
        this);
  }

  /** Creates a command that collects at the slow speed stops when interrupted. */
  public Command collectSlow() {
    return Commands.startEnd(
        () -> {
          runVelocity(collectSlowSpeed.get());
        },
        () -> {
          stop();
        },
        this);
  }
}
