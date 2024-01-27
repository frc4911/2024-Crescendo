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
import edu.wpi.first.wpilibj.AnalogInput;

public class CollectIOReal implements CollectIO {
  private final CANSparkFlex collect;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final AnalogInput beamBreak;
  private final double gearRatio;

  public CollectIOReal(CollectConstants collectConstants) {
    collect = new CANSparkFlex(collectConstants.motorId(), MotorType.kBrushless);
    encoder = collect.getEncoder();
    pidController = collect.getPIDController();
    gearRatio = collectConstants.gearRatio();
    beamBreak = new AnalogInput(collectConstants.sensorId());

    configureDevices();
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / gearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / gearRatio);
    inputs.appliedVolts = collect.getAppliedOutput() * collect.getBusVoltage();
    inputs.currentAmps = collect.getOutputCurrent();
    inputs.beamBreakVoltage = beamBreak.getVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    collect.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * gearRatio,
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
