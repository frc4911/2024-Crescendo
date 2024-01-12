// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.constants;

import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record DriveConstants(
    double trackWidthX,
    double trackWidthY,
    double maxLinearSpeed,
    double wheelRadius,
    double treadWear,
    double driveGearRatio,
    double turnGearRatio,
    int pigeonId,
    ModuleConstants frontLeft,
    ModuleConstants frontRight,
    ModuleConstants backLeft,
    ModuleConstants backRight) {
  public static double TURN_GEAR_RATIO = 150.0 / 7.0;
  public static double L1_GEAR_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);
  public static double L2_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static double L3_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static double L4_GEAR_RATIO = (48.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);

  @RecordBuilder
  public record ModuleConstants(
      String name, int encoderId, int driveMotorId, int turnMotorId, double encoderOffset) {}

  public static record PidValues(double p, double i, double d, double s, double v, double a) {}
}
