// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.shooter;

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
    public double guidePositionRad = 0.0;
    public double guideVelocityRadPerSec = 0.0;

    public double shooterTopAppliedVolts = 0.0;
    public double shooterTopCurrentAmps = 0.0;
    public double shooterBottomAppliedVolts = 0.0;
    public double shooterBottomCurrentAmps = 0.0;
    public double aimerAppliedVolts = 0.0;
    public double aimerCurrentAmps = 0.0;
    public double guideAppliedVolts = 0.0;
    public double guideCurrentAmps = 0.0;

    public double beamBreakValue = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterOutput(double percent) {}

  /** Run the shooter at the specified voltage. */
  public default void setShooterVoltage(double volts) {}

  /** Run shooter in closed loop at the specified velocity. */
  public default void setShooterVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop shooter in open loop. */
  public default void stopShooter() {}

  public default void setAimerOutput(double percent) {}

  /** Run the aimer at the specified voltage. */
  public default void setAimerVoltage(double voltage) {}

  /** Run closed loop to the specified position. */
  public default void setAimerPosition(double position, double ffVolts) {}

  public default void setGuideOutput(double percent) {}

  /** Run the guide at the specified voltage. */
  public default void setGuideVoltage(double voltage) {}

  /** Run guide in closed loop at the specified velocity. */
  public default void setGuideVelocity(double velocityRadPerSec) {}

  /** Stop aimer in open loop. */
  public default void stopAimer() {}

  /** Stop guide in open loop. */
  public default void stopGuide() {}

  /** Set velocity PID constants. */
  public default void configureShooterPID(double kP, double kI, double kD) {}

  public default void configureAimerPID(double kP, double kI, double kD) {}

  public default void configureGuidePID(double kP, double kI, double kD) {}

  public default void configureLimits(double forwardLimit, double backwardLimit) {}
}
