// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.climb;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import javax.inject.Inject;

public final class ClimbIOSim implements ClimbIO {
  private final DCMotorSim sim;
  private final PIDController pid;

  private double appliedVolts = 0.0;

  @Inject
  public ClimbIOSim(ClimbConstants constants) {
    // TODO: determine moment of inertia
    sim = new DCMotorSim(DCMotor.getNeoVortex(2), constants.gearRatio(), 0.004);
    pid = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sim.update(0.02);

    inputs.rightPositionRad = 0.0;
    inputs.rightVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = appliedVolts;
    inputs.leftCurrentAmps = sim.getCurrentDrawAmps();
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
