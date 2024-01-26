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

  private final RelativeEncoder encoder;

  public ClimbIOReal() {
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
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / GEAR_RATIO;

    inputs.appliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.currentAmps = new double[] {right.getOutputCurrent(), left.getOutputCurrent()};
  }

  @Override
  public void stop() {
    right.stopMotor();
  }

  private void configureDevices() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    // TODO: determine follow config
    left.follow(right, false);

    left.setCANTimeout(250);
    right.setCANTimeout(250);

    left.setSmartCurrentLimit(25);
    right.setSmartCurrentLimit(25);

    left.enableVoltageCompensation(12.0);
    right.enableVoltageCompensation(12.0);

    encoder.setPosition(0.0);
    encoder.setMeasurementPeriod(10);
    encoder.setAverageDepth(2);

    left.setCANTimeout(0);
    right.setCANTimeout(0);

    left.burnFlash();
    right.burnFlash();
  }
}
