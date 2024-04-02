// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive.modules;

import com.cyberknights4911.drive.GyroIO;
import com.cyberknights4911.drive.Label;
import com.cyberknights4911.drive.Location;
import com.cyberknights4911.drive.ModuleIO;
import dagger.Module;
import dagger.Provides;

@Module
public interface DriveIOStubModule {

  @Provides
  @Location(label = Label.FrontLeft)
  public static ModuleIO providesFrontLeftModuleIO() {
    return new ModuleIO() {};
  }

  @Provides
  @Location(label = Label.FrontRight)
  public static ModuleIO providesFrontRightModuleIO() {
    return new ModuleIO() {};
  }

  @Provides
  @Location(label = Label.BackLeft)
  public static ModuleIO providesBackLeftModuleIO() {
    return new ModuleIO() {};
  }

  @Provides
  @Location(label = Label.BackRight)
  public static ModuleIO providesBackRightModuleIO() {
    return new ModuleIO() {};
  }

  @Provides
  public static GyroIO providesGyroIO() {
    return new GyroIO() {};
  }
}
