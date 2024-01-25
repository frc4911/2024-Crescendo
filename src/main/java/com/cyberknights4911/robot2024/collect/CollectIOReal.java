// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

public class CollectIOReal implements CollectIO {
  // TODO: modify this value to match that of the actual collector
  private static final double GEAR_RATIO = 1.0;
  private static final double KP = 0;
  private static final double KI = 0;
  private static final double KFF = 0;
  // TODO: determine this via characterization
  private static final double MAX_RPM = 1_000;

  private final CANSparkFlex collect;

  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  public CollectIOReal() {
    collect = new CANSparkFlex(0, MotorType.kBrushless);

    encoder = collect.getEncoder();
    pidController = collect.getPIDController();

    configureDevices();
    configurePidController();
  }

  @Override
  public void setVelocity(double velocity) {
    pidController.setReference(velocity * MAX_RPM, CANSparkFlex.ControlType.kVelocity);
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / GEAR_RATIO;
    inputs.appliedVolts = collect.getAppliedOutput() * collect.getBusVoltage();
    inputs.currentAmps = collect.getOutputCurrent();
  }

  private void configureDevices() {
    collect.restoreFactoryDefaults();

    collect.setCANTimeout(250);
    collect.setSmartCurrentLimit(25);
    collect.enableVoltageCompensation(12.0);

    encoder.setPosition(0.0);
    encoder.setMeasurementPeriod(10);
    encoder.setAverageDepth(2);

    collect.setCANTimeout(0);

    collect.burnFlash();
  }

  private void configurePidController() {
    pidController.setP(KP);
    pidController.setI(KI);
    pidController.setFF(KFF);
    pidController.setOutputRange(-1, 1);
  }
}
