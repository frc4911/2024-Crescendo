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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import javax.inject.Inject;

public final class CollectIOSim implements CollectIO {
  private final DCMotorSim sim;
  private final PIDController pid;

  private final DoubleSolenoidSim left;
  private final DoubleSolenoidSim right;

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Inject
  public CollectIOSim(CollectConstants constants) {
    // TODO: determine moment of inertia
    sim = new DCMotorSim(DCMotor.getNeoVortex(1), constants.collectGearRatio(), 0.004);
    pid = new PIDController(0.0, 0.0, 0.0);
    left =
        new DoubleSolenoidSim(
            PneumaticsModuleType.REVPH,
            constants.solenoidLeftForwardId(),
            constants.solenoidLeftReverseId());
    right =
        new DoubleSolenoidSim(
            PneumaticsModuleType.REVPH,
            constants.solenoidRightForwardId(),
            constants.solenoidRightReverseId());
  }

  @Override
  public void updateInputs(CollectIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.collectPositionRad = sim.getAngularPositionRad();
    inputs.collectVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.collectAppliedVolts = appliedVolts;
    inputs.collectCurrentAmps = sim.getCurrentDrawAmps();

    inputs.leftSolenoid = left.get() == Value.kForward;
    inputs.rightSolenoid = right.get() == Value.kForward;
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
  public void setCollecterPosition(boolean extended) {
    left.set(extended ? Value.kForward : Value.kReverse);
    right.set(extended ? Value.kForward : Value.kReverse);
  }

  @Override
  public void configureCollectPID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
