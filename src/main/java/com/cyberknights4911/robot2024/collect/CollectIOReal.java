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
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class CollectIOReal implements CollectIO {
  private final CANSparkFlex collect;
  private final CANSparkFlex guide;

  private final RelativeEncoder collectEncoder;
  private final RelativeEncoder guideEncoder;

  private final SparkPIDController collectPidController;
  private final SparkPIDController guidePidController;

  private final Solenoid left;
  private final Solenoid right;

  private final AnalogInput beamBreak;
  private final double collectGearRatio;
  private final SparkBurnManager sparkBurnManager;

  public CollectIOReal(CollectConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    collectGearRatio = constants.collectGearRatio();

    collect = new CANSparkFlex(constants.motorId(), MotorType.kBrushless);
    collectEncoder = collect.getEncoder();
    collectPidController = collect.getPIDController();

    guide = new CANSparkFlex(constants.motorIdguide(), MotorType.kBrushless);
    guideEncoder = guide.getEncoder();
    guidePidController = guide.getPIDController();

    left = new Solenoid(PneumaticsModuleType.REVPH, constants.forwardId());
    right = new Solenoid(PneumaticsModuleType.REVPH, constants.reverseId());

    beamBreak = new AnalogInput(constants.sensorId());

    configureDevices();
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.collectPositionRad =
        Units.rotationsToRadians(collectEncoder.getPosition() / collectGearRatio);
    inputs.collectVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(collectEncoder.getVelocity() / collectGearRatio);
    inputs.guidePositionRad = Units.rotationsToRadians(guideEncoder.getPosition());
    inputs.guideVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(guideEncoder.getVelocity());

    inputs.collectAppliedVolts = collect.getAppliedOutput() * collect.getBusVoltage();
    inputs.collectCurrentAmps = collect.getOutputCurrent();
    inputs.guideAppliedVolts = guide.getAppliedOutput() * guide.getBusVoltage();
    inputs.guideCurrentAmps = guide.getOutputCurrent();

    inputs.leftSolenoid = left.get();
    inputs.rightSolenoid = right.get();

    inputs.beamBreakVoltage = beamBreak.getVoltage();
  }

  @Override
  public void setCollectVoltage(double volts) {
    collect.setVoltage(volts);
  }

  @Override
  public void setCollectVelocity(double velocityRadPerSec, double ffVolts) {
    collectPidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * collectGearRatio,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stopCollector() {
    collect.stopMotor();
  }

  @Override
  public void setGuideVelocity(double velocityRadPerSec) {
    guidePidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), ControlType.kVelocity);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    collectPidController.setP(kP, 0);
    collectPidController.setI(kI, 0);
    collectPidController.setD(kD, 0);
    collectPidController.setFF(0, 0);
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configNotLeader(collect);
          SparkConfig.configNotLeader(guide);

          collect.setIdleMode(IdleMode.kBrake);
          guide.setIdleMode(IdleMode.kBrake);

          collect.setSmartCurrentLimit(40);
          guide.setSmartCurrentLimit(40);

          collect.enableVoltageCompensation(12.0);
          guide.enableVoltageCompensation(12.0);

          collectEncoder.setPosition(0.0);
          collectEncoder.setMeasurementPeriod(10);
          collectEncoder.setAverageDepth(2);

          guideEncoder.setPosition(0.0);
          guideEncoder.setMeasurementPeriod(10);
          guideEncoder.setAverageDepth(2);
        },
        collect,
        guide);
  }
}
