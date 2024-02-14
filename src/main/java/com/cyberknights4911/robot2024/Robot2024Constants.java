// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.ConstantsBuilder;
import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.constants.ControlConstantsBuilder;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.constants.DriveConstantsBuilder;
import com.cyberknights4911.constants.DriveConstantsModuleConstantsBuilder;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.robot2024.climb.ClimbConstants;
import com.cyberknights4911.robot2024.climb.ClimbConstantsBuilder;
import com.cyberknights4911.robot2024.collect.CollectConstants;
import com.cyberknights4911.robot2024.collect.CollectConstantsBuilder;
import com.cyberknights4911.robot2024.shooter.ShooterConstants;
import com.cyberknights4911.robot2024.shooter.ShooterConstantsBuilder;
import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.CameraConstantsBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Robot2024Constants {

  private Robot2024Constants() {}

  public static final Constants ROBOT_2024 =
      ConstantsBuilder.builder()
          .name("Robot2024")
          .loopPeriodSecs(0.02)
          .tuningMode(true)
          .logPath(null)
          .mode(Mode.REAL)
          .supplier(Robot2024::new)
          .build();

  public static final ControlConstants CONTROL_CONSTANTS =
      ControlConstantsBuilder.builder().driverPort(0).operatorPort(1).stickDeadband(.1).build();

  public static final DriveConstants DRIVE_CONSTANTS =
      DriveConstantsBuilder.builder()
          .maxLinearSpeed(Units.feetToMeters(17.6))
          .trackWidthX(Units.inchesToMeters(22.75))
          .trackWidthY(Units.inchesToMeters(22.75))
          .wheelRadius(Units.inchesToMeters(1.957237517086368))
          .turnGearRatio(DriveConstants.TURN_GEAR_RATIO)
          .driveGearRatio(DriveConstants.L2_GEAR_RATIO)
          .pigeonId(0)
          .turnFeedBackValues(new PidValues(7.0, 0.0, 0.0))
          .driveFeedBackValues(new PidValues(0.05, 0.0, 0.0))
          .driveFeedForwardValues(new FeedForwardValues(0.1, 0.13))
          .frontLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontLeft")
                  .driveMotorId(1)
                  .turnMotorId(5)
                  .encoderId(1)
                  .encoderOffset(-2.276)
                  .build())
          .frontRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontRight")
                  .driveMotorId(2)
                  .turnMotorId(6)
                  .encoderId(2)
                  .encoderOffset(.561)
                  .build())
          .backLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackLeft")
                  .driveMotorId(3)
                  .turnMotorId(7)
                  .encoderId(3)
                  .encoderOffset(0.272)
                  .build())
          .backRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackRight")
                  .driveMotorId(4)
                  .turnMotorId(8)
                  .encoderId(4)
                  .encoderOffset(.572)
                  .build())
          .build();

  public static ClimbConstants CLIMB_CONSTANTS =
      ClimbConstantsBuilder.builder()
          .motorId1(41)
          .motorId2(42)
          .gearRatio(1.0) // todo: Find gear ratio for climber, and possible climb hold thing.
          .feedBackValues(new PidValues(0, 0, 0))
          .feedForwardValues(new FeedForwardValues(0, 0))
          .build();

  public static CollectConstants COLLECT_CONSTANTS =
      CollectConstantsBuilder.builder()
          .motorId(11)
          .sensorId(
              12) // todo: Beam break sensor, if not dependanat on physical port on the roborio.
          .gearRatio(1.0) // todo: Find gear ratio for collector
          .feedBackValues(new PidValues(0, 0, 0))
          .feedForwardValues(new FeedForwardValues(0, 0))
          .build();

  public static ShooterConstants SHOOTER_CONSTANTS =
      ShooterConstantsBuilder.builder()
          .motorId1(21)
          .motorId2(22)
          .motorId3(23)
          .gearRatio(1.0) // todo: Find gear ratio for shooter
          .feedBackValues(new PidValues(0, 0, 0))
          .feedForwardValues(new FeedForwardValues(0, 0))
          .build();

  // Note: these measurements are for the front right swerve-mounted camera
  // Screw: 11.79 inches back of center, ~6.0 inches from floor, 11.79 inches right of center
  // Camera from screw: 0.82 inches back, 2.24 inches up, 1.06 inches left
  // Camera pitch: -28.125 degrees, Camera yaw: -60
  private static final double SWERVE_MOUNTED_CAMERA_OFFSET_X = Units.inchesToMeters(11.79 - .82);
  private static final double SWERVE_MOUNTED_CAMERA_OFFSET_Y = Units.inchesToMeters(11.79 - 1.06);
  private static final double SWERVE_MOUNTED_CAMERA_OFFSET_Z = Units.inchesToMeters(6.0 + 2.24);
  private static final double SWERVE_MOUNTED_CAMERA_PITCH = Units.degreesToRadians(-28.125);
  private static final double SWERVE_MOUNTED_CAMERA_YAW = Units.degreesToRadians(60);

  public static final CameraConstants CAMERA_CONSTANTS_FRONT_LEFT =
      CameraConstantsBuilder.builder()
          .name("photon1")
          .photonCameraName("Camera_1")
          .robotToCamera(
              new Transform3d(
                  new Translation3d(
                      SWERVE_MOUNTED_CAMERA_OFFSET_X,
                      SWERVE_MOUNTED_CAMERA_OFFSET_Y,
                      SWERVE_MOUNTED_CAMERA_OFFSET_Z),
                  new Rotation3d(0, SWERVE_MOUNTED_CAMERA_PITCH, SWERVE_MOUNTED_CAMERA_YAW)))
          .build();

  public static final CameraConstants CAMERA_CONSTANTS_FRONT_RIGHT =
      CameraConstantsBuilder.builder()
          .name("photon2")
          .photonCameraName("Camera_2")
          .robotToCamera(
              new Transform3d(
                  new Translation3d(
                      SWERVE_MOUNTED_CAMERA_OFFSET_X,
                      -SWERVE_MOUNTED_CAMERA_OFFSET_Y,
                      SWERVE_MOUNTED_CAMERA_OFFSET_Z),
                  new Rotation3d(0, SWERVE_MOUNTED_CAMERA_PITCH, -SWERVE_MOUNTED_CAMERA_YAW)))
          .build();

  // TODO: set the transforms for the back cameras once the mounts have been determined.
  public static final CameraConstants CAMERA_CONSTANTS_BACK_LEFT =
      CameraConstantsBuilder.builder()
          .name("photon3")
          .photonCameraName("Camera_3")
          .robotToCamera(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)))
          .build();

  public static final CameraConstants CAMERA_CONSTANTS_BACK_RIGHT =
      CameraConstantsBuilder.builder()
          .name("photon4")
          .photonCameraName("Camera_4")
          .robotToCamera(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)))
          .build();
}
