// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

  // Adjusting one shooting voltage higher than the other, might make it fly/spin better
  // this is why we have two different methods for each motor

  public void setRightShooterVoltage(double volts) {
    shooterIO.setVoltage(volts);
  }

  public void setLeftShooterVoltage(double volts) {
    shooterIO.setVoltage(volts); // TODO: add L and R to ShooterIO
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
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

  /** Stops the shooter. */
  public void stop() {
    shooterIO.stop();
  }
}
