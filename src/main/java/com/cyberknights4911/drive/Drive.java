// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.constants.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final DriveConstants driveConstants;
  private final double maxAngularSpeedMetersPerSecond;

  private SwerveDriveKinematics kinematics;
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  public Drive(
      Constants constants,
      DriveConstants driveConstants,
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.driveConstants = driveConstants;
    this.gyroIO = gyroIO;

    kinematics = new SwerveDriveKinematics(getModuleTranslations(driveConstants));
    double driveBaseRadius =
        Math.hypot(driveConstants.trackWidthX() / 2.0, driveConstants.trackWidthY() / 2.0);
    maxAngularSpeedMetersPerSecond = driveConstants.maxLinearSpeed() / driveBaseRadius;

    modules[0] = new Module(constants, driveConstants, driveConstants.frontLeft(), flModuleIO);
    modules[1] = new Module(constants, driveConstants, driveConstants.frontRight(), frModuleIO);
    modules[2] = new Module(constants, driveConstants, driveConstants.backLeft(), blModuleIO);
    modules[3] = new Module(constants, driveConstants, driveConstants.backRight(), brModuleIO);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }
    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.
      twist =
          new Twist2d(
              twist.dx, twist.dy, gyroInputs.yawPosition.minus(lastGyroRotation).getRadians());
      lastGyroRotation = gyroInputs.yawPosition;
    }
    // Apply the twist (change since last loop cycle) to the current pose
    pose = pose.exp(twist);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, driveConstants.maxLinearSpeed());

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithX() {
    return Commands.runOnce(
        () -> {
          Rotation2d[] headings = new Rotation2d[4];
          for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations(driveConstants)[i].getAngle();
          }
          kinematics.resetHeadings(headings);
          stop();
        },
        this);
  }

  /**
   * Resets the robot's current pose rotation to be zero. Will not modify robot pose translation.
   */
  public Command zeroPoseToCurrentRotation() {
    return Commands.runOnce(
            () -> setPose(new Pose2d(getPose().getTranslation(), new Rotation2d())), this)
        .ignoringDisable(true);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive(
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  controlConstants.stickDeadband());
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), controlConstants.stickDeadband());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * driveConstants.maxLinearSpeed(),
                  linearVelocity.getY() * driveConstants.maxLinearSpeed(),
                  omega * maxAngularSpeedMetersPerSecond,
                  getRotation()));
        },
        this);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations(DriveConstants driveConstants) {
    return new Translation2d[] {
      new Translation2d(driveConstants.trackWidthX() / 2.0, driveConstants.trackWidthY() / 2.0),
      new Translation2d(driveConstants.trackWidthX() / 2.0, -driveConstants.trackWidthY() / 2.0),
      new Translation2d(-driveConstants.trackWidthX() / 2.0, driveConstants.trackWidthY() / 2.0),
      new Translation2d(-driveConstants.trackWidthX() / 2.0, -driveConstants.trackWidthY() / 2.0)
    };
  }
}
