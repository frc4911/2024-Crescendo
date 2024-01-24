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

  // TODO: modify this value to match that of the actual collector
  private static final double GEAR_RATIO = 1.0;
  private final CANSparkFlex climbLeft;
  private final CANSparkFlex climbRight;

  private final RelativeEncoder encoderRight;
  private final RelativeEncoder encoderLeft;

  public ClimbIOReal() {
    climbLeft = new CANSparkFlex(0, MotorType.kBrushless);
    climbRight = new CANSparkFlex(0, MotorType.kBrushless);

    encoderRight = climbRight.getEncoder();
    encoderLeft = climbLeft.getEncoder();
  }

  @Override
  public void setVoltage(double volts) {
    climbLeft.setVoltage(volts);
    climbRight.setVoltage(volts);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRadRight = Units.rotationsToRadians(encoderRight.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSecRight =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderRight.getVelocity()) / GEAR_RATIO;
    inputs.positionRadLeft = Units.rotationsToRadians(encoderLeft.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSecLeft =
        Units.rotationsPerMinuteToRadiansPerSecond(encoderLeft.getVelocity()) / GEAR_RATIO;
    inputs.appliedVoltsLeft = climbLeft.getAppliedOutput() * climbLeft.getBusVoltage();
    inputs.appliedVoltsRight = climbRight.getAppliedOutput() * climbRight.getBusVoltage();
    inputs.currentAmpsLeft = climbLeft.getOutputCurrent();
    inputs.currentAmpsRight = climbRight.getOutputCurrent();
  }

  private void configureDevices() {
    climbLeft.restoreFactoryDefaults();
    climbRight.restoreFactoryDefaults();

    climbLeft.follow(climbRight, true);
  }
}
