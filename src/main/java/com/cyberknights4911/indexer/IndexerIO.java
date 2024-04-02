// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public double beamBreakVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setOutput(double percent) {}

  public default void setVoltage(double voltage) {}

  /** Run indexer closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Stop indexer in open loop. */
  public default void stop() {}

  /** Set indexer velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
