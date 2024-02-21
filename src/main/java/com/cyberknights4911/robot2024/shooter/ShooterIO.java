// Copyright (c) 2024 FRC 4911
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
    public double shooterTopPositionRad = 0.0;
    public double shooterTopVelocityRadPerSec = 0.0;
    public double shooterBottomPositionRad = 0.0;
    public double shooterBottomVelocityRadPerSec = 0.0;
    public double aimerPositionRad = 0.0;
    public double aimerVelocityRadPerSec = 0.0;
    public double indexerPositionRad = 0.0;
    public double indexerVelocityRadPerSec = 0.0;

    public double shooterTopAppliedVolts = 0.0;
    public double shooterTopCurrentAmps = 0.0;
    public double shooterBottomAppliedVolts = 0.0;
    public double shooterBottomCurrentAmps = 0.0;
    public double aimerAppliedVolts = 0.0;
    public double aimerCurrentAmps = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerCurrentAmps = 0.0;

    public double beamBreakValue = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the shooter at the specified voltage. */
  public default void setShooterVoltage(double volts) {}

  /** Run shooter in closed loop at the specified velocity. */
  public default void setShooterVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop shooter in open loop. */
  public default void stopShooter() {}

  /** Run closed loop to the specified position. */
  public default void setAimerPosition(double position, double ffVolts) {}

  /** Run indexer in closed loop at the specified velocity. */
  public default void setIndexerVelocity(double velocityRadPerSec) {}

  /** Stop aimer in open loop. */
  public default void stopAimer() {}

  /** Stop indexer in open loop. */
  public default void stopIndexer() {}

  /** Set velocity PID constants. */
  public default void configureShooterPID(double kP, double kI, double kD) {}

  public default void configureLimits(double forwardLimit, double backwardLimit) {}
}
