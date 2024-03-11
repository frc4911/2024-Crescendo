// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.cyberknights4911.util.SparkBurnManager;
import com.cyberknights4911.util.SparkConfig;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CollectIOReal implements CollectIO {
  private final CANSparkFlex collect;
  private final RelativeEncoder collectEncoder;
  private final SparkPIDController collectPidController;

  private final DoubleSolenoid left;
  private final DoubleSolenoid right;

  private final AnalogInput beamBreak;
  private final double collectGearRatio;
  private final SparkBurnManager sparkBurnManager;

  public CollectIOReal(CollectConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    collectGearRatio = constants.collectGearRatio();

    collect = new CANSparkFlex(constants.motorCollectRightId(), MotorType.kBrushless);
    collectEncoder = collect.getEncoder();
    collectPidController = collect.getPIDController();

    left =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            constants.solenoidLeftForwardId(),
            constants.solenoidLeftReverseId());
    right =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            constants.solenoidRightForwardId(),
            constants.solenoidRightReverseId());

    beamBreak = new AnalogInput(constants.sensorId());

    configureDevices();
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.collectPositionRad =
        Units.rotationsToRadians(collectEncoder.getPosition() / collectGearRatio);
    inputs.collectVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(collectEncoder.getVelocity() / collectGearRatio);

    inputs.collectAppliedVolts = collect.getAppliedOutput() * collect.getBusVoltage();
    inputs.collectCurrentAmps = collect.getOutputCurrent();

    // inputs.leftSolenoid = left.get() == Value.kForward;
    // inputs.rightSolenoid = right.get() == Value.kForward;

    inputs.beamBreakVoltage = beamBreak.getVoltage();
  }

  @Override
  public void setCollectOutput(double percent) {
    collect.set(percent);
  }

  @Override
  public void setCollectVoltage(double volts) {
    collect.setVoltage(volts);
  }

  @Override
  public void setCollectVelocity(double velocityRadPerSec) {
    collectPidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * collectGearRatio,
        ControlType.kVelocity);
  }

  @Override
  public void stopCollector() {
    collect.stopMotor();
  }

  @Override
  public void setCollecterPosition(boolean extended) {
    left.set(extended ? Value.kForward : Value.kReverse);
    right.set(extended ? Value.kForward : Value.kReverse);
  }

  @Override
  public void configureCollectPID(double kP, double kI, double kD) {
    collectPidController.setP(kP, 0);
    collectPidController.setI(kI, 0);
    collectPidController.setD(kD, 0);
    collectPidController.setFF(0, 0);
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configLeaderFollower(collect);

          collect.setIdleMode(IdleMode.kCoast);
          collect.setSmartCurrentLimit(60);
          collect.enableVoltageCompensation(12.0);

          collectEncoder.setPosition(0.0);
          collectEncoder.setMeasurementPeriod(10);
          collectEncoder.setAverageDepth(2);
        },
        collect);
  }
}
