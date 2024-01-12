// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collect extends SubsystemBase {
  // Collect
  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();

  public Collect(CollectIO collectIO) {
    super();
    this.collectIO = collectIO;
  }

  public void setCollectVoltage(double volts) {
    collectIO.setVoltage(volts);
  }

  @Override
  public void periodic() {
    collectIO.updateInputs(inputs);
    Logger.processInputs("Collect", inputs);
  }
}
