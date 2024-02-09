// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class Module {
  private static final LoggedTunableNumber wheelRadius =
      new LoggedTunableNumber("Drive/Module/WheelRadius");
  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Drive/Module/DriveKp");
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("Drive/Module/DriveKd");
  private static final LoggedTunableNumber driveKs =
      new LoggedTunableNumber("Drive/Module/DriveKs");
  private static final LoggedTunableNumber driveKv =
      new LoggedTunableNumber("Drive/Module/DriveKv");
  private static final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/Module/TurnKp");
  private static final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/Module/TurnKd");

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final DriveConstants driveConstants;
  private final DriveConstants.ModuleConstants moduleConstants;

  private final PIDController driveFeedback;
  private final PIDController turnFeedback;
  private SimpleMotorFeedforward driveFeedforward;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private double lastPositionMeters = 0.0; // Used for delta calculation

  public Module(
      Constants constants,
      DriveConstants driveConstants,
      DriveConstants.ModuleConstants moduleConstants,
      ModuleIO io) {
    this.driveConstants = driveConstants;
    this.moduleConstants = moduleConstants;
    this.io = io;

    wheelRadius.initDefault(Units.inchesToMeters(driveConstants.wheelRadius()));
    driveKp.initDefault(driveConstants.driveFeedBackValues().kP());
    driveKd.initDefault(driveConstants.driveFeedBackValues().kD());
    driveKs.initDefault(driveConstants.driveFeedForwardValues().kS());
    driveKv.initDefault(driveConstants.driveFeedForwardValues().kV());
    turnKp.initDefault(driveConstants.turnFeedBackValues().kP());
    turnKd.initDefault(driveConstants.turnFeedBackValues().kD());

    driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    driveFeedback = new PIDController(driveKp.get(), 0.0, driveKv.get());
    turnFeedback = new PIDController(turnKp.get(), 0.0, turnKd.get());

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + moduleConstants.name(), inputs);

    // Update controllers if tunable numbers have changed
    if (driveKp.hasChanged(hashCode()) || driveKd.hasChanged(hashCode())) {
      driveFeedback.setPID(driveKp.get(), 0.0, driveKd.get());
    }
    if (turnKp.hasChanged(hashCode()) || turnKd.hasChanged(hashCode())) {
      turnFeedback.setPID(turnKp.get(), 0.0, turnKd.get());
    }
    if (driveKs.hasChanged(hashCode()) || driveKv.hasChanged(hashCode())) {
      driveFeedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get());
    }

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }
    boolean LockedDrive = false;
    // Run closed loop turn control
    if (angleSetpoint != null && !LockedDrive) {
      io.setTurnVoltage(
          turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnFeedback.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / driveConstants.wheelRadius();
        io.setDriveVoltage(
            driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return inputs.turnPosition.plus(turnRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * driveConstants.wheelRadius();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * driveConstants.wheelRadius();
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module position delta since the last call to this method. */
  public SwerveModulePosition getPositionDelta() {
    var delta = new SwerveModulePosition(getPositionMeters() - lastPositionMeters, getAngle());
    lastPositionMeters = getPositionMeters();
    return delta;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
