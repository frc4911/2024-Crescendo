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
import edu.wpi.first.math.geometry.Transform2d;
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

  public static Transform2d speakerOpening() {
    return new Transform2d();
  }

  public static Transform2d subWooferIndex() {
    return new Transform2d();
  }
}
