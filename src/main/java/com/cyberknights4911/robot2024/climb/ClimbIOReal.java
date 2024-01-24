// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbIOReal implements ClimbIO {

  private final CANSparkFlex climbLeft;
  private final CANSparkFlex climbRight;

  public ClimbIOReal() {
    climbLeft = new CANSparkFlex(0, MotorType.kBrushless);
    climbRight = new CANSparkFlex(0, MotorType.kBrushless);
  }

  @Override
  public void setVoltage(double volts) {
    climbLeft.setVoltage(volts);
    climbRight.setVoltage(volts);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
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
