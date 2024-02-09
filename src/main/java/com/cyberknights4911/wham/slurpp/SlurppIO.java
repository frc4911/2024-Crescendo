// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.slurpp;

import org.littletonrobotics.junction.AutoLog;

public interface SlurppIO {
  @AutoLog
  public static class SlurppIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SlurppIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {}
}
