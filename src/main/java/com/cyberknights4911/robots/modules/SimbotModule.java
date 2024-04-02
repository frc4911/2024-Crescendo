// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots.modules;

import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.robots.Robot2024;
import dagger.Module;
import dagger.Provides;
import javax.inject.Named;

@Module
public interface SimbotModule {

  @Provides
  public static @Named("TuningMode") boolean providesTuningMode() {
    return true;
  }

  @Provides
  public static @Named("LogPath") String providesLogPath() {
    return null;
  }

  @Provides
  public static @Named("RobotName") String providesName() {
    return "Sim";
  }

  @Provides
  public static Mode providesMode() {
    return Mode.SIM;
  }

  @Provides
  public static RobotContainer providesRobotContainer(Robot2024 hotWheels) {
    return hotWheels;
  }
}
