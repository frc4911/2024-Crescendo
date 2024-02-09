// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.slurpp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class SlurppIOSim implements SlurppIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotorSim sim = new DCMotorSim(DCMotor.getFalcon500(1), 1.5, 0.004);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(SlurppIOInputs inputs) {
    sim.update(LOOP_PERIOD_SECS);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {Math.abs(sim.getCurrentDrawAmps())};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(volts);
  }
}
