// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterIOReal implements ShooterIO {
  private final CANSparkFlex shootLeft;
  private final CANSparkFlex shootRight;

  public ShooterIOReal() {
    shootLeft = new CANSparkFlex(0, MotorType.kBrushless);
    shootRight = new CANSparkFlex(0, MotorType.kBrushless);
  }

  @Override
  public void setVoltage(double volts) {
    shootLeft.setVoltage(volts);
    shootRight.setVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVoltsLeft = shootLeft.getAppliedOutput() * shootLeft.getBusVoltage();
    inputs.appliedVoltsRight = shootRight.getAppliedOutput() * shootRight.getBusVoltage();

    inputs.currentLeftAmps = shootLeft.getOutputCurrent();
    inputs.currentRightAmps = shootRight.getOutputCurrent();

    // inputs.positionRadLeft = shootLeft.getEncoder().getPosition();
    // inputs.positionRadRight = shootRight.getEncoder().getPosition();

    // inputs.velocityRadPerSecLeft = shootLeft.getEncoder().getVelocity();
    // inputs.velocityRadPerSecRight = shootRight.getEncoder().getVelocity();

  }

  private void configureDevices() {
    shootLeft.restoreFactoryDefaults();
    shootRight.restoreFactoryDefaults();

    shootLeft.follow(shootRight, true);
  }
}
