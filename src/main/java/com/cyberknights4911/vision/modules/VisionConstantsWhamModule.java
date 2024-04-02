// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision.modules;

import com.cyberknights4911.util.Field;
import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.CameraConstantsBuilder;
import com.cyberknights4911.vision.VisionConstants;
import com.cyberknights4911.vision.VisionConstantsBuilder;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

@Module
public interface VisionConstantsWhamModule {

  // Note: these measurements are for the front right swerve-mounted camera
  // Screw: 11.79 inches back of center, ~6.0 inches from floor, 11.79 inches right of center
  // Camera from screw: 0.82 inches back, 2.24 inches up, 1.06 inches left
  // Camera pitch: -28.125 degrees, Camera yaw: -60
  public static final double SWERVE_MOUNTED_CAMERA_OFFSET_X = Units.inchesToMeters(11.79 - .82);
  public static final double SWERVE_MOUNTED_CAMERA_OFFSET_Y = Units.inchesToMeters(11.79 - 1.06);
  public static final double SWERVE_MOUNTED_CAMERA_OFFSET_Z = Units.inchesToMeters(6.0 + 2.24);
  public static final double SWERVE_MOUNTED_CAMERA_PITCH = Units.degreesToRadians(-28.125);
  public static final double SWERVE_MOUNTED_CAMERA_YAW = Units.degreesToRadians(60);

  @Provides
  public static VisionConstants providesVisionConstants() {
    return VisionConstantsBuilder.builder()
        .layout(Field.getFieldLayout())
        .maxAmbiguity(0.03)
        .maxValidDistanceMeters(3.0)
        .build();
  }

  @Provides
  @IntoSet
  public static CameraConstants providesCameraConstants() {
    return CameraConstantsBuilder.builder()
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
}
