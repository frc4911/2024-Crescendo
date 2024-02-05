// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.drive.ModuleIO;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkFlex implements ModuleIO {

  private final CANSparkFlex driveMotor;
  private final CANSparkFlex turnMotor;
  private final CANcoder cancoder;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;
  private final DriveConstants driveConstants;

  public ModuleIOSparkFlex(
      SparkOdometryThread sparkOdometryThread,
      DriveConstants driveConstants,
      DriveConstants.ModuleConstants moduleConstants) {
    this.driveConstants = driveConstants;
    driveMotor = new CANSparkFlex(moduleConstants.driveMotorId(), MotorType.kBrushless);
    turnMotor = new CANSparkFlex(moduleConstants.turnMotorId(), MotorType.kBrushless);
    cancoder = new CANcoder(moduleConstants.encoderId());
    absoluteEncoderOffset = new Rotation2d(moduleConstants.encoderOffset());

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    driveMotor.setCANTimeout(250);
    turnMotor.setCANTimeout(250);

    driveEncoder = driveMotor.getEncoder();
    turnRelativeEncoder = turnMotor.getEncoder();

    turnMotor.setInverted(isTurnMotorInverted);
    driveMotor.setSmartCurrentLimit(40);
    turnMotor.setSmartCurrentLimit(30);
    driveMotor.enableVoltageCompensation(12.0);
    turnMotor.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveMotor.setCANTimeout(0);
    turnMotor.setCANTimeout(0);

    driveMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / driveConstants.odometryFrequency()));
    turnMotor.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / driveConstants.odometryFrequency()));
    timestampQueue = sparkOdometryThread.makeTimestampQueue();
    drivePositionQueue = sparkOdometryThread.registerSignal(driveEncoder::getPosition);
    turnPositionQueue = sparkOdometryThread.registerSignal(turnRelativeEncoder::getPosition);

    driveMotor.burnFlash();
    turnMotor.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / driveConstants.driveGearRatio();
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / driveConstants.driveGearRatio();
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / driveConstants.turnGearRatio());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / driveConstants.turnGearRatio();
    inputs.turnAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnMotor.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value) / driveConstants.driveGearRatio())
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / driveConstants.turnGearRatio()))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
