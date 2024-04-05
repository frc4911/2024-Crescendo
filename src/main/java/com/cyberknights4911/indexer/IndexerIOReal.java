// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.indexer;

import com.cyberknights4911.util.SparkBurnManager;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import javax.inject.Inject;

public final class IndexerIOReal implements IndexerIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  private final SparkBurnManager sparkBurnManager;

  @Inject
  public IndexerIOReal(IndexerConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;

    motor = new CANSparkFlex(constants.motorId(), MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidController = motor.getPIDController();

    configureDevices();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP, 0);
    pidController.setI(kI, 0);
    pidController.setD(kD, 0);
    pidController.setFF(0, 0);
  }

  @Override
  public void setOutput(double percent) {
    motor.set(percent);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    pidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), ControlType.kVelocity);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          // SparkConfig.configNotLeader(motor);

          motor.setIdleMode(IdleMode.kCoast);
          motor.setSmartCurrentLimit(60);
          // motor.enableVoltageCompensation(12.0);

          encoder.setPosition(0.0);
          encoder.setMeasurementPeriod(10);
          encoder.setAverageDepth(2);

          motor.setInverted(true);
        },
        motor);
  }
}