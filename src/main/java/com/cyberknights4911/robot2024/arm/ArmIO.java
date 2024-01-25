// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVoltsLeft = 0.0;
    public double appliedVoltsRight = 0.0;
    public double currentAmpsLeft = 0.0;
    public double currentAmpsRight = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the motor at the specified velocity. */
  public default void setVelocity(double velocity) {}

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean enable) {}
}
