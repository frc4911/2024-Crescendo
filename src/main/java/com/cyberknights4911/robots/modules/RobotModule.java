// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robots.modules;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import javax.inject.Named;

@Module
public interface RobotModule {

  @Provides
  public static @Named("LoopPeriod") double providesLoopPeriod() {
    return 0.02;
  }

  @Provides
  public static CommandScheduler providesCommandScheduler() {
    return CommandScheduler.getInstance();
  }
}
