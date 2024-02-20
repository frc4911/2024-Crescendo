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
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

public class ClimbIOReal implements ClimbIO {
  private final CANSparkFlex left;
  private final CANSparkFlex right;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final SparkPIDController leftPidController;
  private final SparkPIDController rightPidController;
  private final double gearRatio;
  private final SparkBurnManager sparkBurnManager;

  public ClimbIOReal(ClimbConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    left = new CANSparkFlex(constants.leftMotorId(), MotorType.kBrushless);
    right = new CANSparkFlex(constants.rightMotorId(), MotorType.kBrushless);

    leftEncoder = left.getEncoder();
    leftPidController = left.getPIDController();
    rightEncoder = right.getEncoder();
    rightPidController = right.getPIDController();
    gearRatio = constants.gearRatio();

    configureDevices();
  }

  @Override
  public void configureLimits(double forwardLimit, double backwardLimit) {
    left.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    left.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    left.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) forwardLimit);
    left.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) backwardLimit);

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
  public void setPositionLeft(double positionRotations) {
    leftPidController.setReference(positionRotations, ControlType.kPosition, 0);
  }

  @Override
  public void setPositionRight(double positionRotations) {
    rightPidController.setReference(positionRotations, ControlType.kPosition, 0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition()) / gearRatio;
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity()) / gearRatio;
    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition()) / gearRatio;
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity()) / gearRatio;

    inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
    inputs.leftCurrentAmps = left.getOutputCurrent();
    inputs.rightAppliedVolts = right.getAppliedOutput() * right.getBusVoltage();
    inputs.rightCurrentAmps = right.getOutputCurrent();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // TODO: set motors to the corresponding brake mode
  }

  @Override
  public void setClimbLock(boolean enable) {
    // TODO: engage the climb lock
  }

  @Override
  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leftPidController.setP(kP, 0);
    leftPidController.setI(kI, 0);
    leftPidController.setD(kD, 0);
    leftPidController.setFF(0, 0);
    rightPidController.setP(kP, 0);
    rightPidController.setI(kI, 0);
    rightPidController.setD(kD, 0);
    rightPidController.setFF(0, 0);
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configLeaderFollower(left);
          SparkConfig.configLeaderFollower(right);

          // TODO: one of these has to be inverted

          left.setIdleMode(IdleMode.kBrake);
          right.setIdleMode(IdleMode.kBrake);
          left.setSmartCurrentLimit(40);
          right.setSmartCurrentLimit(40);
          left.enableVoltageCompensation(12);
          right.enableVoltageCompensation(12);

          rightEncoder.setPosition(0.0);
          rightEncoder.setMeasurementPeriod(10);
          rightEncoder.setAverageDepth(2);
        },
        left,
        right);
  }
}
