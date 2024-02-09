// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham.yeet;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface YeetIO {
  @AutoLog
  public static class YeetIOInputs {
    public Rotation2d wristAbsolutePositionRad = new Rotation2d();
    public Rotation2d wristPositionRad = new Rotation2d();
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;

    public Rotation2d shoulderAbsolutePositionRad = new Rotation2d();
    public Rotation2d shoulderPositionRad = new Rotation2d();
    public double shoulderVelocityRadPerSec = 0.0;
    public double[] shoulderAppliedVolts = new double[3];
    public double[] shoulderCurrentAmps = new double[3];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(YeetIOInputs inputs) {}

  /** Run the wrist motor at the specified voltage. */
  public default void setWristVoltage(double volts) {}

  /** Enable or disable brake mode on the wrist motor. */
  public default void setWristBrakeMode(boolean enable) {}

  /** Run the shoulder motors at the specified voltage. */
  public default void setShoulderVoltage(double volts) {}

  /** Enable or disable brake mode on the shoulder motors. */
  public default void setShoulderBrakeMode(boolean enable) {}
}
