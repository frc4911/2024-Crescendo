// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class ShooterIOSim implements ShooterIO {
  private final DCMotorSim sim;
  private final PIDController pid;

  private double appliedVolts = 0.0;

  public ShooterIOSim(ShooterConstants constants) {
    // TODO: determine moment of inertia
    sim = new DCMotorSim(DCMotor.getNeoVortex(2), constants.gearRatio(), 0.004);
    pid = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(0.02);

    inputs.positionRad = 0.0;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
