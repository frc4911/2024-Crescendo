// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ClimbIOReal implements ClimbIO {

  // TODO: modify this value to match that of the actual climber
  private static final double GEAR_RATIO = 1.0;
  private final CANSparkFlex left;
  private final CANSparkFlex right;

  private final RelativeEncoder encoderRight;
  private final RelativeEncoder encoderLeft;

  public ClimbIOReal() {
    left = new CANSparkFlex(0, MotorType.kBrushless);
    right = new CANSparkFlex(0, MotorType.kBrushless);

    encoderRight = right.getEncoder();
    encoderLeft = left.getEncoder();

    configureDevices();
  }

  @Override
  public void setVoltage(double volts) {
    left.setVoltage(volts);
    right.setVoltage(volts);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRadRight = Units.rotationsToRadians(encoderRight.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSecRight =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderRight.getVelocity()) / GEAR_RATIO;
    inputs.positionRadLeft = Units.rotationsToRadians(encoderLeft.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSecLeft =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderLeft.getVelocity()) / GEAR_RATIO;

    inputs.appliedVoltsLeft = left.getAppliedOutput() * left.getBusVoltage();
    inputs.appliedVoltsRight = right.getAppliedOutput() * right.getBusVoltage();
    inputs.currentAmpsLeft = left.getOutputCurrent();
    inputs.currentAmpsRight = right.getOutputCurrent();
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

    encoderLeft.setPosition(0.0);
    encoderLeft.setMeasurementPeriod(10);
    encoderLeft.setAverageDepth(2);

    encoderRight.setPosition(0.0);
    encoderRight.setMeasurementPeriod(10);
    encoderRight.setAverageDepth(2);

    left.setCANTimeout(0);
    right.setCANTimeout(0);

    left.burnFlash();
    right.burnFlash();
  }
}
