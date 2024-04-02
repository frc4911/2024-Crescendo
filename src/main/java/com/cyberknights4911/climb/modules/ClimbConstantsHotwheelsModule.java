// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.climb.modules;

import com.cyberknights4911.climb.ClimbConstants;
import com.cyberknights4911.climb.ClimbConstantsBuilder;
import com.cyberknights4911.util.PidValues;
import dagger.Module;
import dagger.Provides;

@Module
public interface ClimbConstantsHotwheelsModule {
  @Provides
  public static ClimbConstants providesClimbConstants() {
    return ClimbConstantsBuilder.builder()
        .leftMotorId(41)
        .rightMotorId(42)
        .gearRatio(1.0) // todo: Find gear ratio for climber, and possible climb hold thing.
        .feedBackValues(new PidValues(0.1, 0, 0))
        .build();
  }
}
