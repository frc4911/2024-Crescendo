// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class CollectIOReal implements CollectIO {

  // TODO: modify this value to match that of the actual collector
  private static final double GEAR_RATIO = 1.0;
  private final CANSparkFlex collectLeft;
  private final CANSparkFlex collectRight;

  private final RelativeEncoder encoderRight;
  private final RelativeEncoder encoderLeft;

  public CollectIOReal() {
    collectLeft = new CANSparkFlex(0, MotorType.kBrushless);
    collectRight = new CANSparkFlex(0, MotorType.kBrushless);

    encoderRight = collectRight.getEncoder();
    encoderLeft = collectLeft.getEncoder();

    configureDevices();
  }

  @Override
  public void setVoltage(double volts) {
    collectLeft.setVoltage(volts);
    collectRight.setVoltage(volts);
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.positionRadRight = Units.rotationsToRadians(encoderRight.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSecRight =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderRight.getVelocity()) / GEAR_RATIO;
    inputs.positionRadLeft = Units.rotationsToRadians(encoderLeft.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSecLeft =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderLeft.getVelocity()) / GEAR_RATIO;
    inputs.appliedVoltsLeft = collectLeft.getAppliedOutput() * collectLeft.getBusVoltage();
    inputs.appliedVoltsRight = collectRight.getAppliedOutput() * collectRight.getBusVoltage();
    inputs.currentAmpsLeft = collectLeft.getOutputCurrent();
    inputs.currentAmpsRight = collectRight.getOutputCurrent();
  }

  private void configureDevices() {

    collectLeft.restoreFactoryDefaults();
    collectRight.restoreFactoryDefaults();

    collectLeft.follow(collectRight, true);
  }
}
