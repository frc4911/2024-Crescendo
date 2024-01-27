// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public final class ArmIOReal implements ArmIO {
  private final CANSparkFlex left;
  private final CANSparkFlex right;
  private final Solenoid armSolenoid;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final double gearRatio;

  public ArmIOReal(ArmConstants armConstants) {
    left = new CANSparkFlex(armConstants.motorId1(), MotorType.kBrushless);
    right = new CANSparkFlex(armConstants.motorId2(), MotorType.kBrushless);
    // TODO verify module type
    armSolenoid = new Solenoid(PneumaticsModuleType.REVPH, armConstants.solenoidId());
    encoder = right.getEncoder();
    pidController = right.getPIDController();
    gearRatio = armConstants.gearRatio();

    configureDevices();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / gearRatio;
    // TODO: read solenoid state and write to inputs
    inputs.appliedVolts = right.getAppliedOutput() * right.getBusVoltage();
    inputs.currentAmps = new double[] {right.getOutputCurrent(), left.getOutputCurrent()};
    inputs.solenoidState = armSolenoid.get();
  }

  @Override
  public void setSolenoidState(boolean state) {
    armSolenoid.set(state);
  }

  @Override
  public void configureLimits(double forwardLimit, double backwardLimit) {
    right.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    right.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    right.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) forwardLimit);
    right.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) backwardLimit);
  }

  @Override
  public void setVoltage(double volts) {
    right.setVoltage(volts);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    right.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setOutputRange(-1, 1);
  }

  @Override
  public void setSolenoid(boolean on) {
    // TODO: pass on to actual solenoid
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
