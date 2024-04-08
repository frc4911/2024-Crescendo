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
  private final DCMotorSim aimer;
  private final PIDController aimerPid;

  private double appliedVolts = 0.0;

  @Inject
  public ShooterIOSim(ShooterConstants constants) {
    // TODO: determine moment of inertia
    aimer = new DCMotorSim(DCMotor.getNeoVortex(1), constants.aimerGearRatio(), 0.004);
    aimerPid = new PIDController(0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    aimer.update(0.02);

    inputs.shooterTopPositionRad = 0.0;
    inputs.shooterTopVelocityRadPerSec = aimer.getAngularVelocityRadPerSec();
    inputs.shooterTopAppliedVolts = appliedVolts;
    inputs.shooterTopCurrentAmps = aimer.getCurrentDrawAmps();
  }

  @Override
  public void setShooterVoltage(double volts) {
    appliedVolts = 0.0;
    aimer.setInputVoltage(volts);
  }

  @Override
  public void stopShooter() {
    setShooterVoltage(0.0);
  }

  @Override
  public void configureShooterPID(double kP, double kI, double kD) {
    aimerPid.setPID(kP, kI, kD);
  }
}
