// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import org.littletonrobotics.junction.AutoLog;

public interface CollectIO {

  @AutoLog
  public static class CollectIOInputs {
    public double collectPositionRad = 0.0;
    public double collectVelocityRadPerSec = 0.0;
    public double guidePositionRad = 0.0;
    public double guideVelocityRadPerSec = 0.0;

    public double collectAppliedVolts = 0.0;
    public double collectCurrentAmps = 0.0;
    public double guideAppliedVolts = 0.0;
    public double guideCurrentAmps = 0.0;

    public boolean leftSolenoid = false;
    public boolean rightSolenoid = false;

    public double beamBreakVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CollectIOInputs inputs) {}

  /** Run collector open loop at the specified voltage. */
  public default void setCollectVoltage(double volts) {}

  /** Run collector closed loop at the specified velocity. */
  public default void setCollectVelocity(double velocityRadPerSec) {}

  /** Stop collector in open loop. */
  public default void stopCollector() {}

  /** Run guide closed loop at the specified velocity. */
  public default void setGuideVelocity(double velocityRadPerSec) {}

  /** Stop guide in open loop. */
  public default void stopGuide() {}

  /** Set collect velocity PID constants. */
  public default void configureCollectPID(double kP, double kI, double kD) {}

  /** Set guide velocity PID constants. */
  public default void configureGuidePID(double kP, double kI, double kD) {}
}
