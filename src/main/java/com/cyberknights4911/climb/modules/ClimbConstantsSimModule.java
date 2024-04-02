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
public interface ClimbConstantsSimModule {
  @Provides
  public static ClimbConstants providesClimbConstants() {
    return ClimbConstantsBuilder.builder()
        .leftMotorId(0)
        .rightMotorId(0)
        .gearRatio(1.0)
        .feedBackValues(new PidValues(0, 0, 0))
        .build();
  }
}
