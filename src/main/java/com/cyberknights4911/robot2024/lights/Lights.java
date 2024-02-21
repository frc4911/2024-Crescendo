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

  private Hue hue = Hue.Red;
  private Pattern pattern = Pattern.SINELON;

  public Lights(LightsIO lightsIO) {
    this.lightsIO = lightsIO;
  }

  public void setMode(Hue hue) {
    this.hue = hue;
  }

  public void setPattern(Pattern pattern) {
    this.pattern = pattern;
  }

  @Override
  public void periodic() {
    double percentHue = ((double) hue.hueDegrees) / 360.0;
    lightsIO.setHueVoltage(5.0 * percentHue);

    double patternPercent = ((double) pattern.number) / 20.0;
    lightsIO.setPatternVoltage(5.0 * patternPercent);
  }

  public enum Hue {
    Red(0),
    Orange(35),
    Green(100),
    Blue(240),
    Purple(275),
    Pink(300);

    final int hueDegrees;

    Hue(int hueDegrees) {
      this.hueDegrees = hueDegrees;
    }
  }

  public enum Pattern {
    DEFAULT(0),
    CYLON(1),
    FLASH(2),
    CONFETTI(3),
    SINELON(4),
    BPM(5),
    JUGGLE(6);

    final int number;

    Pattern(int number) {
      this.number = number;
    }
  }
}
