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
import com.cyberknights4911.drive.ModuleIOSim;
import dagger.Binds;
import dagger.Module;
import dagger.Provides;

@Module
public interface DriveIOSimModule {

  @Binds
  @Location(label = Label.FrontLeft)
  public abstract ModuleIO bindsFrontLeftModuleIO(ModuleIOSim sim);

  @Binds
  @Location(label = Label.FrontRight)
  public abstract ModuleIO bindsFrontRightModuleIO(ModuleIOSim sim);

  @Binds
  @Location(label = Label.BackLeft)
  public abstract ModuleIO bindsBackLeftModuleIO(ModuleIOSim sim);

  @Binds
  @Location(label = Label.BackRight)
  public abstract ModuleIO bindsBackRightModuleIO(ModuleIOSim sim);

  @Provides
  public static GyroIO providesGyroIO() {
    return new GyroIO() {};
  }
}
