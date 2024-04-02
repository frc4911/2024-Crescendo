// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.util;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

@Module
public interface UtilModule {
  @Provides
  public static Alliance providesAlliance() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
  }
}
