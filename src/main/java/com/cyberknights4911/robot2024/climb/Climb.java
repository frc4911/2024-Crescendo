// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  public Climb(ClimbIO climbIO) {
    super();
    this.climbIO = climbIO;
  }

  public void setClimbVoltage(double volts) {
    climbIO.setVoltage(volts);
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(inputs);

    Logger.processInputs("Climb", inputs);
  }
}
