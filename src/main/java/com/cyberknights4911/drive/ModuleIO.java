package com.cyberknights4911.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  void updateInputs(ModuleIOInputs inputs);

  /** Run the drive motor at the specified voltage. */
  void setDriveVoltage(double volts);

  /** Run the turn motor at the specified voltage. */
  void setTurnVoltage(double volts);

  /** Enable or disable brake mode on the drive motor. */
  void setDriveBrakeMode(boolean enable);

  /** Enable or disable brake mode on the turn motor. */
  void setTurnBrakeMode(boolean enable);
}
