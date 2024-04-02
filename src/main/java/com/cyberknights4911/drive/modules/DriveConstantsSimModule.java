// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive.modules;

import com.cyberknights4911.drive.DriveConstants;
import com.cyberknights4911.drive.DriveConstants.ModuleConstants;
import com.cyberknights4911.drive.DriveConstantsBuilder;
import com.cyberknights4911.drive.DriveConstantsModuleConstantsBuilder;
import com.cyberknights4911.drive.Label;
import com.cyberknights4911.drive.Location;
import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import dagger.Module;
import dagger.Provides;
import edu.wpi.first.math.util.Units;

@Module
public interface DriveConstantsSimModule {

  @Provides
  public static DriveConstants providesDriveConstants() {
    return DriveConstantsBuilder.builder()
        .maxLinearSpeed(4.5)
        .trackWidthX(Units.inchesToMeters(22.75))
        .trackWidthY(Units.inchesToMeters(22.75))
        .wheelRadius(Units.inchesToMeters(2))
        .turnGearRatio(DriveConstants.TURN_GEAR_RATIO)
        .driveGearRatio(DriveConstants.L3_GEAR_RATIO)
        .pigeonId(0)
        .turnFeedBackValues(new PidValues(0.0, 0.0, 0.0))
        .driveFeedBackValues(new PidValues(0.0, 0.0, 0.0))
        .driveFeedForwardValues(new FeedForwardValues(0.0, 0.0))
        .build();
  }

  @Provides
  @Location(label = Label.FrontLeft)
  public static ModuleConstants providesFrontLeftModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("FrontLeft")
        .driveMotorId(0)
        .turnMotorId(0)
        .encoderId(0)
        .encoderOffset(0.0 - Math.PI)
        .build();
  }

  @Provides
  @Location(label = Label.FrontRight)
  public static ModuleConstants providesFrontRightModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("FrontRight")
        .driveMotorId(0)
        .turnMotorId(0)
        .encoderId(0)
        .encoderOffset(0.0 - Math.PI)
        .build();
  }

  @Provides
  @Location(label = Label.BackLeft)
  public static ModuleConstants providesBackLeftModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("BackLeft")
        .driveMotorId(0)
        .turnMotorId(0)
        .encoderId(0)
        .encoderOffset(0.0 - Math.PI)
        .build();
  }

  @Provides
  @Location(label = Label.BackRight)
  public static ModuleConstants providesBackRightModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("BackRight")
        .driveMotorId(0)
        .turnMotorId(0)
        .encoderId(0)
        .encoderOffset(0.0 - Math.PI)
        .build();
  }
}
