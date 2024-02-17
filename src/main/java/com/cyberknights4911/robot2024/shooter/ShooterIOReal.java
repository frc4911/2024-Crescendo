// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import com.cyberknights4911.util.SparkBurnManager;
import com.cyberknights4911.util.SparkConfig;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;

public class ShooterIOReal implements ShooterIO {
  private final CANSparkFlex left;
  private final CANSparkFlex right;
  private final CANSparkFlex guideRoller;
  private final CANSparkFlex aimer;
  private final CANSparkFlex indexer;

  private final RelativeEncoder encoder;
  private final AnalogInput beamBreak;
  private final SparkPIDController pidController;
  private final double gearRatio;
  private final SparkBurnManager sparkBurnManager;

  public ShooterIOReal(ShooterConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    left = new CANSparkFlex(constants.motorId1(), MotorType.kBrushless);
    right = new CANSparkFlex(constants.motorId2(), MotorType.kBrushless);
    guideRoller = new CANSparkFlex(constants.motorIdguide(), MotorType.kBrushless);
    beamBreak = new AnalogInput(constants.sensorId());

    // new motor for shooter, instead of piston, used to aim the shooter at the specified angle
    aimer = new CANSparkFlex(constants.motorId3(), MotorType.kBrushless);

    // motor for the indexer and moving notes to the shooter to shoot
    indexer = new CANSparkFlex(constants.motorIdindex(), MotorType.kBrushless);

    encoder = right.getEncoder();
    pidController = right.getPIDController();
    gearRatio = constants.gearRatio();
    // TODO: aim encoder?

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
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configLeaderFollower(left);
          SparkConfig.configLeaderFollower(right);

          // TODO: determine follow config
          left.follow(right, true);

          left.setSmartCurrentLimit(40);
          right.setSmartCurrentLimit(40);
          left.enableVoltageCompensation(12);
          right.enableVoltageCompensation(12);

          encoder.setPosition(0.0);
          encoder.setMeasurementPeriod(10);
          encoder.setAverageDepth(2);
        },
        left,
        right);
  }
}
