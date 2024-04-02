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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.UncheckedIOException;
import java.util.ArrayList;
import javax.inject.Inject;

public final class Field {
  private static Alert noAprilTagLayoutAlert =
      new Alert("No AprilTag layout file found. Update VisionConstants.", AlertType.WARNING);

  private final Alliance alliance;
  private final AprilTagFieldLayout fieldLayout;
  private final Translation2d speakerOpening;
  private final Pose2d subWooferIndex;
  private final Rotation2d ampOpeningAngle;
  private final Rotation2d forwardAngle;

  @Inject
  public Field(Alliance alliance) {
    this.alliance = alliance;
    fieldLayout = createFieldLayout();

    if (alliance == Alliance.Red) {
      speakerOpening = new Translation2d(652.73, 218.42);
    } else {
      speakerOpening = new Translation2d(-1.50, 218.42);
    }

    if (alliance == Alliance.Red) {
      subWooferIndex = new Pose2d(new Translation2d(602.73, 218.42), new Rotation2d());
    } else {
      // TODO: get values for blue alliance
      subWooferIndex = new Pose2d();
    }

    // Same angle regardless of alliance
    ampOpeningAngle = new Rotation2d(Math.PI / 2);

    if (alliance == Alliance.Red) {
      forwardAngle = new Rotation2d(Math.PI / 2);
    } else {
      forwardAngle = new Rotation2d();
    }
  }

  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }

  public Translation2d speakerOpening() {
    return speakerOpening;
  }

  public Pose2d subWooferIndex() {
    return subWooferIndex;
  }

  public Rotation2d ampOpeningAngle() {
    return ampOpeningAngle;
  }

  public Rotation2d forwardAngle() {
    return forwardAngle;
  }

  private AprilTagFieldLayout createFieldLayout() {
    try {
      noAprilTagLayoutAlert.set(false);
      return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    } catch (UncheckedIOException e) {
      noAprilTagLayoutAlert.set(true);
      return new AprilTagFieldLayout(
          new ArrayList<>(), Units.feetToMeters(12), Units.feetToMeters(12));
    }
  }
}
