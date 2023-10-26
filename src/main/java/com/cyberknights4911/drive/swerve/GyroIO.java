package com.cyberknights4911.drive.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
