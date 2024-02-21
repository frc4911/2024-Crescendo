// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.lights;

import edu.wpi.first.wpilibj.AnalogOutput;

public final class LightsIOReal implements LightsIO {

  private final AnalogOutput output;

  public LightsIOReal(LightsConstants constants) {
    output = new AnalogOutput(constants.analogOutputId());
  }

  public void setSignalVoltage(double voltage) {
    output.setVoltage(voltage);
  }
}
