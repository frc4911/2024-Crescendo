// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

public class CollectIOReal implements CollectIO {
  // TODO: modify this value to match that of the actual collector
  private static final double GEAR_RATIO = 1.0;

  private final CANSparkFlex collect;

  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  public CollectIOReal() {
    collect = new CANSparkFlex(0, MotorType.kBrushless);

    encoder = collect.getEncoder();
    pidController = collect.getPIDController();

    configureDevices();
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = collect.getAppliedOutput() * collect.getBusVoltage();
    inputs.currentAmps = collect.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    collect.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    collect.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP, 0);
    pidController.setI(kI, 0);
    pidController.setD(kD, 0);
    pidController.setFF(0, 0);
  }

  private void configureDevices() {
    collect.restoreFactoryDefaults();

    collect.setIdleMode(IdleMode.kBrake);
    collect.setCANTimeout(250);
    collect.setSmartCurrentLimit(25);
    collect.enableVoltageCompensation(12.0);

    collect.setCANTimeout(0);

    collect.burnFlash();
  }
}
