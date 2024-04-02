// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive.modules;

import com.cyberknights4911.drive.DriveConstants;
import com.cyberknights4911.drive.Label;
import com.cyberknights4911.drive.Location;
import com.cyberknights4911.drive.ModuleIO;
import com.cyberknights4911.logging.LoggedTunableNumberFactory;
import dagger.Module;
import dagger.Provides;

@Module
public interface DriveModule {

  @Provides
  @Location(label = Label.FrontLeft)
  public static com.cyberknights4911.drive.Module providesFrontLeftModule(
      DriveConstants driveConstants,
      LoggedTunableNumberFactory numberFactory,
      @Location(label = Label.FrontLeft) DriveConstants.ModuleConstants moduleConstants,
      @Location(label = Label.FrontLeft) ModuleIO io) {
    return new com.cyberknights4911.drive.Module(
        driveConstants, numberFactory, moduleConstants, io);
  }

  @Provides
  @Location(label = Label.FrontRight)
  public static com.cyberknights4911.drive.Module providesFrontRightModule(
      DriveConstants driveConstants,
      LoggedTunableNumberFactory numberFactory,
      @Location(label = Label.FrontRight) DriveConstants.ModuleConstants moduleConstants,
      @Location(label = Label.FrontRight) ModuleIO io) {
    return new com.cyberknights4911.drive.Module(
        driveConstants, numberFactory, moduleConstants, io);
  }

  @Provides
  @Location(label = Label.BackLeft)
  public static com.cyberknights4911.drive.Module providesBackLeftModule(
      DriveConstants driveConstants,
      LoggedTunableNumberFactory numberFactory,
      @Location(label = Label.BackLeft) DriveConstants.ModuleConstants moduleConstants,
      @Location(label = Label.BackLeft) ModuleIO io) {
    return new com.cyberknights4911.drive.Module(
        driveConstants, numberFactory, moduleConstants, io);
  }

  @Provides
  @Location(label = Label.BackRight)
  public static com.cyberknights4911.drive.Module providesBackRightModule(
      DriveConstants driveConstants,
      LoggedTunableNumberFactory numberFactory,
      @Location(label = Label.BackRight) DriveConstants.ModuleConstants moduleConstants,
      @Location(label = Label.BackRight) ModuleIO io) {
    return new com.cyberknights4911.drive.Module(
        driveConstants, numberFactory, moduleConstants, io);
  }
}
