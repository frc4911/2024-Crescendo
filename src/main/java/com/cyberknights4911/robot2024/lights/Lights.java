// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.lights;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class Lights extends SubsystemBase {
  private final LightsIO lightsIO;

  private Mode message = Mode.Red;

  public Lights(LightsIO lightsIO) {
    this.lightsIO = lightsIO;
  }

  public void setMode(Mode mode) {
    this.message = mode;
  }

  @Override
  public void periodic() {
    double percent = ((double) message.hueDegrees) / 360.0;
    lightsIO.setSignalVoltage(5.0 * percent);
  }

  public enum Mode {
    Red(0),
    Orange(35),
    Green(100),
    Blue(240),
    Purple(275),
    Pink(300);

    final int hueDegrees;

    Mode(int hueDegrees) {
      this.hueDegrees = hueDegrees;
    }
  }
}
