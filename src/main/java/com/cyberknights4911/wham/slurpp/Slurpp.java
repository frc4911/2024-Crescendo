// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.slurpp;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Slurpp extends SubsystemBase {

  private final SlurppIO slurppIO;
  private final SlurppIOInputsAutoLogged inputs = new SlurppIOInputsAutoLogged();

  public Slurpp(SlurppIO splurppIO) {
    super();
    this.slurppIO = splurppIO;
  }

  @Override
  public void periodic() {
    slurppIO.updateInputs(inputs);
    Logger.processInputs("Slurpp", inputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      slurppIO.setVoltage(0);
    }
  }
}
