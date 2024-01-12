// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double positionRadLeft = 0.0;
    public double positionRadRight = 0.0;
    public double velocityRadPerSecLeft = 0.0;
    public double velocityRadPerSecRight = 0.0;
    public double appliedVoltsLeft = 0.0;
    public double appliedVoltsRight = 0.0;
    public double currentAmpsRight = 0.0;
    public double currentAmpsLeft = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {}
}
