// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.yeet;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class YeetIOReal implements YeetIO {
  private static final double SHOULDER_GEAR_RATIO = 1.0;
  private static final double WRIST_GEAR_RATIO = 1.0;
  private static final String CANIVORE = "CANivore";

  private final TalonFX wristTalon;
  private final StatusSignal<Double> wristPosition;
  private final StatusSignal<Double> wristVelocity;
  private final StatusSignal<Double> wristAppliedVolts;
  private final StatusSignal<Double> wristCurrent;
  private final StatusSignal<Double> wristAbsolutePosition;
  private final StatusSignal<Double> wristEncoderVelocity;

  private final TalonFX shoulderTalon1;
  private final TalonFX shoulderTalon2;
  private final TalonFX shoulderTalon3;
  private final StatusSignal<Double> shoulderPosition;
  private final StatusSignal<Double> shoulderVelocity;
  private final StatusSignal<Double> shoulderAppliedVolts1;
  private final StatusSignal<Double> shoulderCurrent1;
  private final StatusSignal<Double> shoulderAppliedVolts2;
  private final StatusSignal<Double> shoulderCurrent2;
  private final StatusSignal<Double> shoulderAppliedVolts3;
  private final StatusSignal<Double> shoulderCurrent3;
  private final StatusSignal<Double> shoulderAbsolutePosition;
  private final StatusSignal<Double> shoulderEncoderVelocity;

  private final CANcoder shoulderCancoder;
  private final CANcoder wristCancoder;

  private final Rotation2d shoulderEncoderOffset;
  private final Rotation2d wristEncoderOffset;

  public YeetIOReal() {
    wristTalon = new TalonFX(13, CANIVORE);
    wristCancoder = new CANcoder(11, CANIVORE);
    shoulderTalon1 = new TalonFX(0, CANIVORE);
    shoulderTalon2 = new TalonFX(0, CANIVORE);
    shoulderTalon3 = new TalonFX(0, CANIVORE);
    shoulderCancoder = new CANcoder(10, CANIVORE);

    wristPosition = wristTalon.getPosition();
    wristVelocity = wristTalon.getVelocity();
    wristAppliedVolts = wristTalon.getMotorVoltage();
    wristCurrent = wristTalon.getStatorCurrent();
    wristAbsolutePosition = wristCancoder.getAbsolutePosition();
    wristEncoderVelocity = wristCancoder.getVelocity();

    shoulderPosition = shoulderTalon1.getPosition();
    shoulderVelocity = shoulderTalon1.getVelocity();
    shoulderAppliedVolts1 = shoulderTalon1.getMotorVoltage();
    shoulderCurrent1 = shoulderTalon1.getStatorCurrent();
    shoulderAppliedVolts2 = shoulderTalon2.getMotorVoltage();
    shoulderCurrent2 = shoulderTalon2.getStatorCurrent();
    shoulderAppliedVolts3 = shoulderTalon3.getMotorVoltage();
    shoulderCurrent3 = shoulderTalon3.getStatorCurrent();

    shoulderAbsolutePosition = shoulderCancoder.getAbsolutePosition();
    shoulderEncoderVelocity = shoulderCancoder.getVelocity();

    shoulderEncoderOffset = new Rotation2d();
    wristEncoderOffset = new Rotation2d();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, wristAbsolutePosition, shoulderAbsolutePosition);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristCurrent,
        wristEncoderVelocity,
        shoulderPosition,
        shoulderVelocity,
        shoulderAppliedVolts1,
        shoulderCurrent1,
        shoulderAppliedVolts2,
        shoulderCurrent2,
        shoulderAppliedVolts3,
        shoulderCurrent3,
        shoulderEncoderVelocity);

    wristTalon.optimizeBusUtilization();
    wristCancoder.optimizeBusUtilization();
    shoulderTalon1.optimizeBusUtilization();
    shoulderTalon2.optimizeBusUtilization();
    shoulderTalon3.optimizeBusUtilization();
    shoulderCancoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(YeetIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        wristPosition,
        wristVelocity,
        wristAppliedVolts,
        wristCurrent,
        wristAbsolutePosition,
        shoulderAbsolutePosition);

    inputs.wristAbsolutePositionRad =
        Rotation2d.fromRotations(wristAbsolutePosition.getValueAsDouble())
            .minus(wristEncoderOffset);
    inputs.wristPositionRad =
        Rotation2d.fromRotations(wristPosition.getValueAsDouble() / WRIST_GEAR_RATIO);
    inputs.wristVelocityRadPerSec =
        Units.rotationsToRadians(wristVelocity.getValueAsDouble() / WRIST_GEAR_RATIO);
    inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = wristCurrent.getValueAsDouble();
    inputs.shoulderAbsolutePositionRad =
        Rotation2d.fromRotations(shoulderAbsolutePosition.getValueAsDouble())
            .minus(shoulderEncoderOffset);
  }

  @Override
  public void setShoulderBrakeMode(boolean enable) {
    // TODO Auto-generated method stub
  }

  @Override
  public void setShoulderVoltage(double volts) {
    shoulderTalon1.setControl(new VoltageOut(volts));
  }

  @Override
  public void setWristBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    wristTalon.getConfigurator().apply(config);
  }

  @Override
  public void setWristVoltage(double volts) {
    wristTalon.setControl(new VoltageOut(volts));
  }
}
