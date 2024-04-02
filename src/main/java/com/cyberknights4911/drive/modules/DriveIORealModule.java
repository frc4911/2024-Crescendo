// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive.modules;

import com.cyberknights4911.drive.DriveConstants.ModuleConstants;
import com.cyberknights4911.drive.GyroIO;
import com.cyberknights4911.drive.GyroIOPigeon2;
import com.cyberknights4911.drive.Label;
import com.cyberknights4911.drive.Location;
import com.cyberknights4911.drive.ModuleIO;
import com.cyberknights4911.drive.ModuleIOTalonFXFactory;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;

@Module
public interface DriveIORealModule {

  @Provides
  @Location(label = Label.FrontLeft)
  public static ModuleIO providesFrontLeftModuleIO(
      @Location(label = Label.FrontLeft) ModuleConstants moduleConstants,
      ModuleIOTalonFXFactory factory) {
    return factory.create(moduleConstants);
  }

  @Provides
  @Location(label = Label.FrontRight)
  public static ModuleIO providesFrontRightModuleIO(
      @Location(label = Label.FrontRight) ModuleConstants moduleConstants,
      ModuleIOTalonFXFactory factory) {
    return factory.create(moduleConstants);
  }

  @Provides
  @Location(label = Label.BackLeft)
  public static ModuleIO providesBackLeftModuleIO(
      @Location(label = Label.BackLeft) ModuleConstants moduleConstants,
      ModuleIOTalonFXFactory factory) {
    return factory.create(moduleConstants);
  }

  @Provides
  @Location(label = Label.BackRight)
  public static ModuleIO providesBackRightModuleIO(
      @Location(label = Label.BackRight) ModuleConstants moduleConstants,
      ModuleIOTalonFXFactory factory) {
    return factory.create(moduleConstants);
  }

  @Binds
  public abstract GyroIO bindsGyroIO(GyroIOPigeon2 real);
}
