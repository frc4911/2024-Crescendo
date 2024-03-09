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
  private final CANSparkFlex guide;

  private final RelativeEncoder shooterTopEncoder;
  private final RelativeEncoder shooterBottomEncoder;
  private final RelativeEncoder aimerEncoder;
  private final RelativeEncoder guideEncoder;

  private final SparkPIDController shooterPidController;
  private final SparkPIDController aimerPidController;
  private final SparkPIDController guidePidController;

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

    guide = new CANSparkFlex(constants.guideMotorId(), MotorType.kBrushless);
    guideEncoder = guide.getEncoder();
    guidePidController = guide.getPIDController();

    beamBreak = new AnalogInput(constants.sensorId());

    configureDevices();
  }

  @Override
  public void setShooterOutput(double percent) {
    shooterTop.set(percent);
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
  public void setAimerOutput(double percent) {
    aimer.set(percent);
  }

  @Override
  public void setAimerVoltage(double voltage) {
    aimer.setVoltage(voltage);
  }

  @Override
  public void setAimerPosition(double positionRadians, double ffVolts) {
    // aimerPidController.setReference(
    //     Units.radiansToDegrees(positionRadians) * aimerGearRatio,
    //     ControlType.kSmartMotion,
    //     0,
    //     ffVolts,
    //     ArbFFUnits.kVoltage);

    aimerPidController.setReference(
        Units.radiansToRotations(positionRadians) * aimerGearRatio, ControlType.kPosition);
  }

  @Override
  public void setGuideOutput(double percent) {
    guide.set(percent);
  }

  @Override
  public void setGuideVoltage(double voltage) {
    guide.setVoltage(voltage);
  }

  @Override
  public void setGuideVelocity(double velocityRadiansPerSecond) {
    guidePidController.setReference(
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
    inputs.guidePositionRad = Units.rotationsToRadians(guideEncoder.getPosition());
    inputs.guideVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(guideEncoder.getVelocity());

    inputs.shooterTopAppliedVolts = shooterTop.getAppliedOutput() * shooterTop.getBusVoltage();
    inputs.shooterTopCurrentAmps = shooterTop.getOutputCurrent();
    inputs.shooterBottomAppliedVolts =
        shooterBottom.getAppliedOutput() * shooterBottom.getBusVoltage();
    inputs.shooterBottomCurrentAmps = shooterBottom.getOutputCurrent();
    inputs.aimerAppliedVolts = aimer.getAppliedOutput() * aimer.getBusVoltage();
    inputs.aimerCurrentAmps = aimer.getOutputCurrent();
    inputs.guideAppliedVolts = guide.getAppliedOutput() * guide.getBusVoltage();
    inputs.guideCurrentAmps = guide.getOutputCurrent();

    inputs.beamBreakValue = beamBreak.getVoltage();
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
  public void stopGuide() {
    guide.stopMotor();
  }

  @Override
  public void configureShooterPID(double kP, double kI, double kD) {
    shooterPidController.setP(kP, 0);
    shooterPidController.setI(kI, 0);
    shooterPidController.setD(kD, 0);
    shooterPidController.setFF(0, 0);
  }

  @Override
  public void configureAimerPID(double kP, double kI, double kD) {
    aimerPidController.setP(kP, 0);
    aimerPidController.setI(kI, 0);
    aimerPidController.setD(kD, 0);
    aimerPidController.setFF(0, 0);
  }

  @Override
  public void configureGuidePID(double kP, double kI, double kD) {
    guidePidController.setP(kP, 0);
    guidePidController.setI(kI, 0);
    guidePidController.setD(kD, 0);
    guidePidController.setFF(0, 0);
  }

  @Override
  public void configureLimits(double forwardLimit, double backwardLimit) {
    // aimer.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    // aimer.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    // aimer.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) forwardLimit);
    // aimer.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) backwardLimit);
  }

  private void configureDevices() {
    sparkBurnManager.maybeBurnConfig(
        () -> {
          SparkConfig.configLeaderFollower(shooterBottom);
          SparkConfig.configLeaderFollower(shooterTop);
          SparkConfig.configNotLeader(aimer);
          SparkConfig.configNotLeader(guide);

          shooterBottom.follow(shooterTop, true);
          guide.setInverted(true);
          // aimer.setInverted(true);

          shooterTop.setSmartCurrentLimit(80);
          shooterBottom.setSmartCurrentLimit(80);
          aimer.setSmartCurrentLimit(80);
          guide.setSmartCurrentLimit(60);
          shooterTop.enableVoltageCompensation(12);
          shooterBottom.enableVoltageCompensation(12);
          aimer.enableVoltageCompensation(12);
          guide.enableVoltageCompensation(12);

          shooterTop.setIdleMode(IdleMode.kBrake);
          shooterBottom.setIdleMode(IdleMode.kBrake);
          aimer.setIdleMode(IdleMode.kBrake);
          guide.setIdleMode(IdleMode.kBrake);

          shooterTopEncoder.setPosition(0.0);
          shooterTopEncoder.setMeasurementPeriod(10);
          shooterTopEncoder.setAverageDepth(2);
          shooterBottomEncoder.setPosition(0.0);
          shooterBottomEncoder.setMeasurementPeriod(10);
          shooterBottomEncoder.setAverageDepth(2);
          aimerEncoder.setPosition(0.0);
          aimerEncoder.setMeasurementPeriod(10);
          aimerEncoder.setAverageDepth(2);
          guideEncoder.setPosition(0.0);
          guideEncoder.setMeasurementPeriod(10);
          guideEncoder.setAverageDepth(2);
        },
        shooterTop,
        shooterBottom,
        aimer,
        guide);
  }
}
