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
import com.cyberknights4911.util.SparkBurnManager;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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

  private final CANSparkFlex driveSparkMax;
  private final CANSparkFlex turnSparkMax;
  private final CANcoder cancoder;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;

  private final StatusSignal<Double> turnAbsolutePosition;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;
  private final DriveConstants driveConstants;
  private final SparkBurnManager sparkBurnManager;

  public ModuleIOSparkFlex(
      DriveConstants driveConstants,
      DriveConstants.ModuleConstants moduleConstants,
      SparkBurnManager sparkBurnManager) {
    this.driveConstants = driveConstants;
    this.sparkBurnManager = sparkBurnManager;
    driveSparkMax = new CANSparkFlex(moduleConstants.driveMotorId(), MotorType.kBrushless);
    turnSparkMax = new CANSparkFlex(moduleConstants.turnMotorId(), MotorType.kBrushless);
    cancoder = new CANcoder(moduleConstants.encoderId());
    absoluteEncoderOffset = new Rotation2d(moduleConstants.encoderOffset());

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, turnAbsolutePosition);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    sparkBurnManager.maybeBurnConfig(
        () -> {
          turnSparkMax.setInverted(isTurnMotorInverted);
          driveSparkMax.setSmartCurrentLimit(40);
          turnSparkMax.setSmartCurrentLimit(30);
          driveSparkMax.enableVoltageCompensation(12.0);
          turnSparkMax.enableVoltageCompensation(12.0);

          driveEncoder.setPosition(0.0);
          driveEncoder.setMeasurementPeriod(10);
          driveEncoder.setAverageDepth(2);

          turnRelativeEncoder.setPosition(0.0);
          turnRelativeEncoder.setMeasurementPeriod(10);
          turnRelativeEncoder.setAverageDepth(2);
        },
        driveSparkMax,
        turnSparkMax);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / driveConstants.driveGearRatio();
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / driveConstants.driveGearRatio();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(
            turnRelativeEncoder.getPosition() / driveConstants.turnGearRatio());
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / driveConstants.turnGearRatio();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
