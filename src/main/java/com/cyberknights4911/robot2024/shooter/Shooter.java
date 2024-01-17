// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    super();
    this.shooterIO = shooterIO;
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
    shooterIO.updateInputs(null);

    Logger.processInputs("Shooter", inputs);
  }
}
