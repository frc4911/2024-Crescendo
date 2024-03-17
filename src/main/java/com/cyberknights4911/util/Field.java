// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.util;

import com.cyberknights4911.logging.Alert;
import com.cyberknights4911.logging.Alert.AlertType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.io.UncheckedIOException;
import java.util.ArrayList;

public final class Field {
  private static Alert noAprilTagLayoutAlert =
      new Alert("No AprilTag layout file found. Update VisionConstants.", AlertType.WARNING);

  private Field() {}

  public static AprilTagFieldLayout getFieldLayout() {
    try {
      noAprilTagLayoutAlert.set(false);
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    } catch (UncheckedIOException e) {
      noAprilTagLayoutAlert.set(true);
      return new AprilTagFieldLayout(
          new ArrayList<>(), Units.feetToMeters(12), Units.feetToMeters(12));
    }
  }

  public static Translation2d speakerOpening() {
    if (Alliance.isRed()) {
      return new Translation2d(652.73, 218.42);
    } else {
      return new Translation2d(-1.50, 218.42);
    }
  }

  public static Pose2d subWooferIndex() {
    if (Alliance.isRed()) {
      return new Pose2d(new Translation2d(602.73, 218.42), new Rotation2d());
    } else {
      // TODO: get values for blue alliance
      return new Pose2d();
    }
  }

  public static Rotation2d ampOpeningAngle() {
    // Same angle regardless of alliance
    return new Rotation2d(Math.PI / 2);
  }

  public static Rotation2d forwardAngle() {
    if (Alliance.isRed()) {
      return new Rotation2d(Math.PI / 2);
    } else {
      return new Rotation2d();
    }
  }
}
