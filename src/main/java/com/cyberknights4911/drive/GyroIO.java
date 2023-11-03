package com.cyberknights4911.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
