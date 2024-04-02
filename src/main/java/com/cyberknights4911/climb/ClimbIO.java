// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double leftPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightPositionRad = 0.0;
    public double rightVelocityRadPerSec = 0.0;

    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Set the left climber to the desired position. */
  public default void setPositionLeft(double position) {}

  /** Set the right climber to the desired position. */
  public default void setPositionRight(double position) {}

  /** Enable or disable brake mode on the motor. */
  public default void setBrakeMode(boolean enable) {}

  /** Enable or disable the climb lock. */
  public default void setClimbLock(boolean enable) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  public default void configureLimits(double forwardLimit, double backwardLimit) {}
}
