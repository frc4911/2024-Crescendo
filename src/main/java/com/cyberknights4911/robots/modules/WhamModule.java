// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots.modules;

import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.robots.Wham;
import dagger.Module;
import dagger.Provides;
import javax.inject.Named;

@Module
public interface WhamModule {

  @Provides
  public static @Named("TuningMode") boolean providesTuningMode() {
    return true;
  }

  @Provides
  public static @Named("LogPath") String providesLogPath() {
    return "/media/sda1/logs";
  }

  @Provides
  public static @Named("RobotName") String providesName() {
    return "WHAM!";
  }

  @Provides
  public static Mode providesMode() {
    return Mode.REAL;
  }

  @Provides
  public static RobotContainer providesRobotContainer(Wham wham) {
    return wham;
  }
}
