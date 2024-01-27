// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV");

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private SimpleMotorFeedforward feedforward;

  public Shooter(ShooterConstants constants, ShooterIO shooterIO) {
    super();
    this.shooterIO = shooterIO;
    kS.initDefault(constants.feedForwardValues().kS());
    kV.initDefault(constants.feedForwardValues().kV());
    kP.initDefault(constants.feedBackValues().kP());
    kD.initDefault(constants.feedBackValues().kD());
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());
    shooterIO.configurePID(kP.get(), 0.0, kD.get());
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(null);
    Logger.processInputs("Shooter", inputs);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      shooterIO.configurePID(kP.get(), 0.0, kD.get());
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
    shooterIO.setVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRpm() {
    double velocityRpm = Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    Logger.recordOutput("Shooter/VelocityRPM", velocityRpm);
    return velocityRpm;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    shooterIO.setVoltage(volts);
  }

  /** Returns the average velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  /** Stops the shooter. */
  public void stop() {
    shooterIO.stop();
  }
}
