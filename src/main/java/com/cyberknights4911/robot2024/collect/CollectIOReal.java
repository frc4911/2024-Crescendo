// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class CollectIOReal implements CollectIO {
  // TODO: modify this value to match that of the actual collector
  private static final double GEAR_RATIO = 1.0;

  private final CANSparkMax collectLeft;
  private final CANSparkMax collectRight;

  private final RelativeEncoder encoder;

  public CollectIOReal() {
    collectLeft = new CANSparkMax(0, MotorType.kBrushless);
    collectRight = new CANSparkMax(0, MotorType.kBrushless);

    encoder = collectRight.getEncoder();
  }

  @Override
  public void setVoltage(double volts) {
    collectLeft.setVoltage(volts);
    collectRight.setVoltage(volts);
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / GEAR_RATIO;
    inputs.appliedVoltsLeft = collectLeft.getAppliedOutput() * collectLeft.getBusVoltage();
    inputs.appliedVoltsRight = collectRight.getAppliedOutput() * collectRight.getBusVoltage();
    inputs.currentAmpsLeft = collectLeft.getOutputCurrent();
    inputs.currentAmpsRight = collectRight.getOutputCurrent();
  }
}
