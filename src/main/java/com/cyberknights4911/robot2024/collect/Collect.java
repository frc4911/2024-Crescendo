// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
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

  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();
  private final SimpleMotorFeedforward feedforward;

  public Collect(CollectIO collectIO) {
    super();
    this.collectIO = collectIO;
    switch (Constants.get().mode()) {
      case REAL:
      case REPLAY:
        feedforward = new SimpleMotorFeedforward(0.0, 0.0);
        collectIO.configurePID(0.0, 0.0, 0.0);
        break;
      case SIM:
        feedforward = new SimpleMotorFeedforward(0.0, 0.0);
        collectIO.configurePID(0.0, 0.0, 0.0);
        break;
      default:
        feedforward = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    collectIO.updateInputs(inputs);
    Logger.processInputs("Collect", inputs);
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
