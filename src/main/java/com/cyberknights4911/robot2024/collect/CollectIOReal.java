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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CollectIOReal implements CollectIO {
  private final CANSparkFlex collect;
  private final CANSparkFlex guideRoller;
  private final DoubleSolenoid extend;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final AnalogInput beamBreak;
  private final double gearRatio;
  private final SparkBurnManager sparkBurnManager;

  public CollectIOReal(CollectConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    collect = new CANSparkFlex(constants.motorId(), MotorType.kBrushless);
    guideRoller = new CANSparkFlex(constants.motorIdguide(), MotorType.kBrushless);
    extend =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH, constants.forwardId(), constants.reverseId());
    encoder = collect.getEncoder();
    pidController = collect.getPIDController();
    gearRatio = constants.gearRatio();
    beamBreak = new AnalogInput(constants.sensorId());

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
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configNotLeader(collect);

          collect.setIdleMode(IdleMode.kBrake);
          collect.setSmartCurrentLimit(40);
          collect.enableVoltageCompensation(12.0);
          encoder.setMeasurementPeriod(10);
          encoder.setAverageDepth(2);
        },
        collect);
  }
}
