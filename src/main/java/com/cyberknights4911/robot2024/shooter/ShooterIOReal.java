// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

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

public class ShooterIOReal implements ShooterIO {
  private final CANSparkFlex shooterTop;
  private final CANSparkFlex shooterBottom;
  private final CANSparkFlex aimer;
  private final CANSparkFlex indexer;

  private final RelativeEncoder shooterTopEncoder;
  private final RelativeEncoder shooterBottomEncoder;
  private final RelativeEncoder aimerEncoder;
  private final RelativeEncoder indexerEncoder;

  private final SparkPIDController shooterPidController;
  private final SparkPIDController aimerPidController;
  private final SparkPIDController indexerPidController;

  private final AnalogInput beamBreak;

  private final double aimerGearRatio;
  private final SparkBurnManager sparkBurnManager;

  public ShooterIOReal(ShooterConstants constants, SparkBurnManager sparkBurnManager) {
    this.sparkBurnManager = sparkBurnManager;
    aimerGearRatio = constants.aimerGearRatio();

    shooterTop = new CANSparkFlex(constants.shooterMotorTopId(), MotorType.kBrushless);
    shooterBottom = new CANSparkFlex(constants.shooterMotorBottomId(), MotorType.kBrushless);
    shooterTopEncoder = shooterTop.getEncoder();
    shooterBottomEncoder = shooterBottom.getEncoder();
    shooterPidController = shooterTop.getPIDController();

    // Used to aim the shooter at the specified angle
    aimer = new CANSparkFlex(constants.aimerMotorId(), MotorType.kBrushless);
    aimerEncoder = aimer.getEncoder();
    aimerPidController = aimer.getPIDController();

    // motor for the indexer and moving notes to the shooter to shoot
    indexer = new CANSparkFlex(constants.indexerMotorId(), MotorType.kBrushless);
    indexerEncoder = indexer.getEncoder();
    indexerPidController = indexer.getPIDController();

    beamBreak = new AnalogInput(constants.sensorId());

    configureDevices();
  }

  @Override
  public void setShooterVoltage(double volts) {
    shooterTop.setVoltage(volts);
  }

  @Override
  public void setShooterVelocity(double velocityRadiansPerSecond, double ffVolts) {
    shooterPidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadiansPerSecond),
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setAimerPosition(double positionRadians, double ffVolts) {
    aimerPidController.setReference(
        Units.radiansToDegrees(positionRadians),
        ControlType.kSmartMotion,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setIndexerVelocity(double velocityRadiansPerSecond) {
    indexerPidController.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadiansPerSecond),
        ControlType.kVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterTopPositionRad = Units.rotationsToRadians(shooterTopEncoder.getPosition());
    inputs.shooterTopVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterTopEncoder.getVelocity());
    inputs.shooterBottomPositionRad = Units.rotationsToRadians(shooterBottomEncoder.getPosition());
    inputs.shooterBottomVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(shooterBottomEncoder.getVelocity());
    inputs.aimerPositionRad = Units.rotationsToRadians(aimerEncoder.getPosition()) / aimerGearRatio;
    inputs.aimerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(aimerEncoder.getVelocity()) / aimerGearRatio;
    inputs.indexerPositionRad = Units.rotationsToRadians(indexerEncoder.getPosition());
    inputs.indexerVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(indexerEncoder.getVelocity());

    inputs.shooterTopAppliedVolts = shooterTop.getAppliedOutput() * shooterTop.getBusVoltage();
    inputs.shooterTopCurrentAmps = shooterTop.getOutputCurrent();
    inputs.shooterBottomAppliedVolts =
        shooterBottom.getAppliedOutput() * shooterBottom.getBusVoltage();
    inputs.shooterBottomCurrentAmps = shooterBottom.getOutputCurrent();
    inputs.aimerAppliedVolts = aimer.getAppliedOutput() * aimer.getBusVoltage();
    inputs.aimerCurrentAmps = aimer.getOutputCurrent();
    inputs.indexerAppliedVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();
    inputs.indexerCurrentAmps = indexer.getOutputCurrent();

    inputs.beamBreakVoltage = beamBreak.getVoltage();
  }

  @Override
  public void stopShooter() {
    shooterTop.stopMotor();
  }

  @Override
  public void stopAimer() {
    aimer.stopMotor();
  }

  @Override
  public void stopIndexer() {
    indexer.stopMotor();
  }

  @Override
  public void configureShooterPID(double kP, double kI, double kD) {
    shooterPidController.setP(kP, 0);
    shooterPidController.setI(kI, 0);
    shooterPidController.setD(kD, 0);
    shooterPidController.setFF(0, 0);
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configLeaderFollower(shooterBottom);
          SparkConfig.configLeaderFollower(shooterTop);
          SparkConfig.configNotLeader(aimer);
          SparkConfig.configNotLeader(indexer);

          shooterBottom.follow(shooterTop, true);

          shooterTop.setSmartCurrentLimit(40);
          shooterBottom.setSmartCurrentLimit(40);
          aimer.setSmartCurrentLimit(40);
          indexer.setSmartCurrentLimit(30);
          shooterTop.enableVoltageCompensation(12);
          shooterBottom.enableVoltageCompensation(12);
          aimer.enableVoltageCompensation(12);
          indexer.enableVoltageCompensation(12);

          shooterTop.setIdleMode(IdleMode.kCoast);
          shooterBottom.setIdleMode(IdleMode.kCoast);
          aimer.setIdleMode(IdleMode.kBrake);
          indexer.setIdleMode(IdleMode.kBrake);

          shooterTopEncoder.setPosition(0.0);
          shooterTopEncoder.setMeasurementPeriod(10);
          shooterTopEncoder.setAverageDepth(2);
          shooterBottomEncoder.setPosition(0.0);
          shooterBottomEncoder.setMeasurementPeriod(10);
          shooterBottomEncoder.setAverageDepth(2);
          aimerEncoder.setPosition(0.0);
          aimerEncoder.setMeasurementPeriod(10);
          aimerEncoder.setAverageDepth(2);
          indexerEncoder.setPosition(0.0);
          indexerEncoder.setMeasurementPeriod(10);
          indexerEncoder.setAverageDepth(2);
        },
        shooterTop,
        shooterBottom,
        aimer,
        indexer);
  }
}
