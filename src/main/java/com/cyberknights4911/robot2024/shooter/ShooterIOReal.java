// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOReal implements ShooterIO {
  // TODO: modify this value to match that of the actual collector
  private static final double GEAR_RATIO = 1.0;
  private final CANSparkFlex left;
  private final CANSparkFlex right;

  private final RelativeEncoder encoder;

  public ShooterIOReal() {
    left = new CANSparkFlex(0, MotorType.kBrushless);
    right = new CANSparkFlex(0, MotorType.kBrushless);

    encoder = right.getEncoder();

    configureDevices();
  }

  @Override
  public void setVoltage(double volts) {
    right.setVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / GEAR_RATIO;

    inputs.appliedVoltsLeft = left.getAppliedOutput() * left.getBusVoltage();
    inputs.appliedVoltsRight = right.getAppliedOutput() * right.getBusVoltage();
    inputs.currentLeftAmps = left.getOutputCurrent();
    inputs.currentRightAmps = right.getOutputCurrent();
  }

  private void configureDevices() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setCANTimeout(250);
    right.setCANTimeout(250);

    left.setSmartCurrentLimit(25);
    right.setSmartCurrentLimit(25);

    left.enableVoltageCompensation(12.0);
    right.enableVoltageCompensation(12.0);

    left.follow(right, true);

    encoder.setPosition(0.0);
    encoder.setMeasurementPeriod(10);
    encoder.setAverageDepth(2);

    left.setCANTimeout(0);
    right.setCANTimeout(0);

    left.burnFlash();
    right.burnFlash();
  }
}
