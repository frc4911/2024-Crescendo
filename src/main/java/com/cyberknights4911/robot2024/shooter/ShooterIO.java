// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVoltsLeft = 0.0;
    public double appliedVoltsRight = 0.0;
    public double currentLeftAmps = 0.0;
    public double currentRightAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setVoltage(double volts) {}
}
