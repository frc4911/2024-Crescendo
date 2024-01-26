// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Arm extends SubsystemBase {
  private final ArmIO armIO;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO armIO) {
    super();
    this.armIO = armIO;
  }

  public void setVoltage(double volts) {
    armIO.setVoltage(volts);
  }

  @Override
  public void periodic() {
    armIO.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }
}
