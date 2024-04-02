// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import javax.inject.Inject;

public final class ShooterIOSim implements ShooterIO {
  private final DCMotorSim sim;
  private final PIDController pid;

  private double appliedVolts = 0.0;

  @Inject
  public ShooterIOSim(ShooterConstants constants) {
    // TODO: determine moment of inertia
    sim = new DCMotorSim(DCMotor.getNeoVortex(2), constants.aimerGearRatio(), 0.004);
    pid = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(0.02);

    inputs.shooterTopPositionRad = 0.0;
    inputs.shooterTopVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.shooterTopAppliedVolts = appliedVolts;
    inputs.shooterTopCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setShooterVoltage(double volts) {
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stopShooter() {
    setShooterVoltage(0.0);
  }

  @Override
  public void configureShooterPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
