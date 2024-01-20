// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final VisionIO visionIO;

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
  }
}
