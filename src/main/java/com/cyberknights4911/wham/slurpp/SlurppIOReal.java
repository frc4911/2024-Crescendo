// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.slurpp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public final class SlurppIOReal implements SlurppIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX talon;
  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> current;

  public SlurppIOReal() {
    talon = new TalonFX(20, "CANivore");

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    current = talon.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    talon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SlurppIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {current.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talon.getConfigurator().apply(config);
  }
}
