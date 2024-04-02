// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.collect;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import javax.inject.Inject;

public final class CollectIOSim implements CollectIO {
  private final DCMotorSim sim;
  private final AnalogInputSim beamBreakSim;
  private final PIDController pid;

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Inject
  public CollectIOSim(CollectConstants constants) {
    // TODO: determine moment of inertia
    sim = new DCMotorSim(DCMotor.getNeoVortex(1), constants.collectGearRatio(), 0.004);
    pid = new PIDController(0.0, 0.0, 0.0);
    beamBreakSim = new AnalogInputSim(0);
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.beamBreakVoltage = beamBreakSim.getVoltage();
    inputs.collectPositionRad = sim.getAngularPositionRad();
    inputs.collectVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.collectAppliedVolts = appliedVolts;
    inputs.collectCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setCollectVoltage(double volts) {
    closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setCollectVelocity(double velocityRadPerSec) {
    closedLoop = true;
    pid.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void stopCollector() {
    setCollectVoltage(0.0);
  }

  @Override
  public void configureCollectPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
