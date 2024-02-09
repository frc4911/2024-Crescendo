// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import com.cyberknights4911.util.SparkBurnManager;
import com.cyberknights4911.util.SparkConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

public class ClimbIOReal implements ClimbIO {
  private final CANSparkFlex left;
  private final CANSparkFlex right;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final double gearRatio;
  private final SparkBurnManager sparkBurnManager;

  public ClimbIOReal(ClimbConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    left = new CANSparkFlex(constants.motorId1(), MotorType.kBrushless);
    right = new CANSparkFlex(constants.motorId2(), MotorType.kBrushless);
    encoder = right.getEncoder();
    pidController = right.getPIDController();
    gearRatio = constants.gearRatio();

    configureDevices();
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
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition()) / gearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / gearRatio;

    inputs.appliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.currentAmps = new double[] {right.getOutputCurrent(), left.getOutputCurrent()};
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // TODO: set motors to the corresponding brake mode
  }

  @Override
  public void setClimbLock(boolean enable) {
    // TODO: set the climb lock
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
          left.follow(right, false);

          left.setIdleMode(IdleMode.kBrake);
          right.setIdleMode(IdleMode.kBrake);
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
