// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive.advanced;

import static edu.wpi.first.units.Units.Volts;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.drive.GyroIO;
import com.cyberknights4911.drive.GyroIOInputsAutoLogged;
import com.cyberknights4911.drive.ModuleIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final DriveConstants driveConstants;
  private final double maxAngularSpeedMetersPerSecond;
  private final SysIdRoutine sysId;

  private final SwerveDriveKinematics kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator;

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
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    double driveBaseRadius =
        Math.hypot(driveConstants.trackWidthX() / 2.0, driveConstants.trackWidthY() / 2.0);
    maxAngularSpeedMetersPerSecond = driveConstants.maxLinearSpeed() / driveBaseRadius;

    modules[0] = new Module(constants, driveConstants, driveConstants.frontLeft(), flModuleIO);
    modules[1] = new Module(constants, driveConstants, driveConstants.frontRight(), frModuleIO);
    modules[2] = new Module(constants, driveConstants, driveConstants.backLeft(), blModuleIO);
    modules[3] = new Module(constants, driveConstants, driveConstants.backRight(), brModuleIO);
    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();
    SparkMaxOdometryThread.getInstance().start();

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
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
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
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

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    Logger.recordOutput("SwerveStates/Measured", states);
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  public Pose2d getPose() {
    Pose2d pose = poseEstimator.getEstimatedPosition();
    Logger.recordOutput("Odometry/Robot", pose);
    return pose;
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
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

  ChassisSpeeds createChassisSpeeds(
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
            controlConstants.stickDeadband());
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
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
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        linearVelocity.getX() * driveConstants.maxLinearSpeed(),
        linearVelocity.getY() * driveConstants.maxLinearSpeed(),
        omega * maxAngularSpeedMetersPerSecond,
        getRotation());
  }

  public PointToAngleDrive pointToPointDrive(
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double x,
      double y) {
    return PointToAngleDrive.createDriveFacingPoint(
        this, controlConstants, xSupplier, ySupplier, x, y);
  }

  public PointToAngleDrive pointToAngleDrive(
      ControlConstants controlConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double angleRadians) {
    return PointToAngleDrive.createDriveFacingFixedAngle(
        this, controlConstants, xSupplier, ySupplier, angleRadians);
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
          Logger.recordOutput("Drive/Joystick/Omega", omegaSupplier.getAsDouble());
          runVelocity(createChassisSpeeds(controlConstants, xSupplier, ySupplier, omegaSupplier));
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
