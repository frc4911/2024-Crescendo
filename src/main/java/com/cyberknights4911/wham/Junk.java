// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Junk {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  public Junk() {

    motor = new CANSparkFlex(31, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidController = motor.getPIDController();
    pidController.setP(0.1, 0);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void stop() {
    motor.stopMotor();
  }
}
