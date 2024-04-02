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
public interface DriveConstantsWhamModule {

  @Provides
  public static DriveConstants providesDriveConstants() {
    return DriveConstantsBuilder.builder()
        .maxLinearSpeed(4.5)
        .trackWidthX(Units.inchesToMeters(22.75))
        .trackWidthY(Units.inchesToMeters(22.75))
        .wheelRadius(Units.inchesToMeters(1.941349158748698))
        .turnGearRatio(DriveConstants.TURN_GEAR_RATIO)
        .driveGearRatio(DriveConstants.L1_GEAR_RATIO)
        .pigeonId(0)
        .canBusId("rio")
        .turnFeedBackValues(new PidValues(7.0, 0.0, 0.0))
        .driveFeedBackValues(new PidValues(0.05, 0.0, 0.0))
        .driveFeedForwardValues(new FeedForwardValues(0.1, 0.13))
        .build();
  }

  @Provides
  @Location(label = Label.FrontLeft)
  public static ModuleConstants providesFrontLeftModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("FrontLeft")
        .driveMotorId(2)
        .turnMotorId(6)
        .encoderId(1)
        .encoderOffset(0.360 - Math.PI)
        .build();
  }

  @Provides
  @Location(label = Label.FrontRight)
  public static ModuleConstants providesFrontRightModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("FrontRight")
        .driveMotorId(1)
        .turnMotorId(5)
        .encoderId(0)
        .encoderOffset(2.769 - Math.PI)
        .build();
  }

  @Provides
  @Location(label = Label.BackLeft)
  public static ModuleConstants providesBackLeftModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("BackLeft")
        .driveMotorId(3)
        .turnMotorId(7)
        .encoderId(2)
        .encoderOffset(-2.946 - Math.PI)
        .build();
  }

  @Provides
  @Location(label = Label.BackRight)
  public static ModuleConstants providesBackRightModuleConstants() {
    return DriveConstantsModuleConstantsBuilder.builder()
        .name("BackRight")
        .driveMotorId(4)
        .turnMotorId(8)
        .encoderId(3)
        .encoderOffset(1.804 - Math.PI)
        .build();
  }
}
