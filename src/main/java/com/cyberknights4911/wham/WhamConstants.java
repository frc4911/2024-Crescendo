// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.ConstantsBuilder;
import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.constants.ControlConstantsBuilder;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.constants.DriveConstantsBuilder;
import com.cyberknights4911.constants.DriveConstantsModuleConstantsBuilder;
import com.cyberknights4911.logging.Alert;
import com.cyberknights4911.logging.Alert.AlertType;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.CameraConstantsBuilder;
import com.cyberknights4911.vision.VisionConstants;
import com.cyberknights4911.vision.VisionConstantsBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.UncheckedIOException;
import java.util.ArrayList;

public final class WhamConstants {
  private WhamConstants() {}

  public static final Constants WHAM =
      ConstantsBuilder.builder()
          .name("Wham")
          .loopPeriodSecs(0.02)
          .tuningMode(true)
          .logPath("/media/sda1/logs")
          .mode(Mode.REAL)
          .supplier(Wham::new)
          .build();

  public static final ControlConstants CONTROL_CONSTANTS =
      ControlConstantsBuilder.builder().driverPort(0).operatorPort(1).stickDeadband(.1).build();

  public static final DriveConstants DRIVE_CONSTANTS =
      DriveConstantsBuilder.builder()
          .maxLinearSpeed(4.5)
          .trackWidthX(Units.inchesToMeters(22.75))
          .trackWidthY(Units.inchesToMeters(22.75))
          .wheelRadius(Units.inchesToMeters(1.941349158748698))
          .turnGearRatio(DriveConstants.TURN_GEAR_RATIO)
          .driveGearRatio(DriveConstants.L1_GEAR_RATIO)
          .pigeonId(0)
          .turnFeedBackValues(new PidValues(7.0, 0.0, 0.0))
          .driveFeedBackValues(new PidValues(0.05, 0.0, 0.0))
          .driveFeedForwardValues(new FeedForwardValues(0.1, 0.13))
          .frontLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontLeft")
                  .driveMotorId(2)
                  .turnMotorId(6)
                  .encoderId(1)
                  .encoderOffset(0.360 - Math.PI)
                  .build())
          .frontRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontRight")
                  .driveMotorId(1)
                  .turnMotorId(5)
                  .encoderId(0)
                  .encoderOffset(2.769 - Math.PI)
                  .build())
          .backLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackLeft")
                  .driveMotorId(3)
                  .turnMotorId(7)
                  .encoderId(2)
                  .encoderOffset(-2.946 - Math.PI)
                  .build())
          .backRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackRight")
                  .driveMotorId(4)
                  .turnMotorId(8)
                  .encoderId(3)
                  .encoderOffset(1.804 - Math.PI)
                  .build())
          .build();

  private static Alert noAprilTagLayoutAlert =
      new Alert("No AprilTag layout file found. Update VisionConstants.", AlertType.WARNING);

  private static AprilTagFieldLayout getFieldLayout() {
    try {
      noAprilTagLayoutAlert.set(false);
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    } catch (UncheckedIOException e) {
      noAprilTagLayoutAlert.set(true);
      return new AprilTagFieldLayout(
          new ArrayList<>(), Units.feetToMeters(12), Units.feetToMeters(12));
    }
  }

  public static final VisionConstants VISION_CONSTANTS =
      VisionConstantsBuilder.builder()
          .layout(getFieldLayout())
          .maxAmbiguity(0.03)
          .maxValidDistanceMeters(3.0)
          .build();

  // Note: these measurements are for the front right swerve-mounted camera
  // Screw: 11.79 inches back of center, ~6.0 inches from floor, 11.79 inches right of center
  // Camera from screw: 0.82 inches back, 2.24 inches up, 1.06 inches left
  // Camera pitch: -28.125 degrees, Camera yaw: -60
  private static final double SWERVE_MOUNTED_CAMERA_OFFSET_X = Units.inchesToMeters(11.79 - .82);
  private static final double SWERVE_MOUNTED_CAMERA_OFFSET_Y = Units.inchesToMeters(11.79 - 1.06);
  private static final double SWERVE_MOUNTED_CAMERA_OFFSET_Z = Units.inchesToMeters(6.0 + 2.24);
  private static final double SWERVE_MOUNTED_CAMERA_PITCH = Units.degreesToRadians(28.125);
  private static final double SWERVE_MOUNTED_CAMERA_YAW = Units.degreesToRadians(60);

  public static final CameraConstants CAMERA_CONSTANTS_FRONT_RIGHT =
      CameraConstantsBuilder.builder()
          .name("photon4")
          .photonCameraName("Camera_4")
          .robotToCamera(
              new Transform3d(
                  new Translation3d(
                      SWERVE_MOUNTED_CAMERA_OFFSET_X,
                      -SWERVE_MOUNTED_CAMERA_OFFSET_Y,
                      SWERVE_MOUNTED_CAMERA_OFFSET_Z),
                  new Rotation3d(0, -SWERVE_MOUNTED_CAMERA_PITCH, -SWERVE_MOUNTED_CAMERA_YAW)))
          .build();
}
