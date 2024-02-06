// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

public class ShooterIOReal implements ShooterIO {
  private final CANSparkFlex left;
  private final CANSparkFlex right;
  private final CANSparkFlex aimer;

  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final double gearRatio;

  public ShooterIOReal(ShooterConstants constants) {
    left = new CANSparkFlex(constants.motorId1(), MotorType.kBrushless);
    right = new CANSparkFlex(constants.motorId2(), MotorType.kBrushless);

    aimer =
        new CANSparkFlex(
            constants.motorId3(),
            MotorType
                .kBrushless); // new motor for shooter, instead of piston, used to aim the shooter
    // at the specified angle

    encoder = right.getEncoder();
    pidController = right.getPIDController();
    gearRatio = constants.gearRatio();
    // todo: aim encoder?

    configureDevices();
  }

  @Override
  public void setVoltage(double volts) {
    right.setVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / gearRatio;

    inputs.appliedVolts = right.getAppliedOutput() * right.getBusVoltage();
    inputs.currentAmps = new double[] {right.getOutputCurrent(), left.getOutputCurrent()};
  }

  @Override
  public void stop() {
    right.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pidController.setP(kP, 0);
    pidController.setI(kI, 0);
    pidController.setD(kD, 0);
    pidController.setFF(0, 0);
  }

  private void configureDevices() {
    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setCANTimeout(250);
    right.setCANTimeout(250);

    left.setSmartCurrentLimit(25);
    right.setSmartCurrentLimit(25);

    // todo: aim limits (if needed)

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
