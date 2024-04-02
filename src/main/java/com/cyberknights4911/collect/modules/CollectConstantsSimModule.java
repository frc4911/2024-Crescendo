// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.collect.modules;

import com.cyberknights4911.collect.CollectConstants;
import com.cyberknights4911.collect.CollectConstantsBuilder;
import com.cyberknights4911.util.PidValues;
import dagger.Module;
import dagger.Provides;

@Module
public interface CollectConstantsSimModule {
  @Provides
  public static CollectConstants providesCollectConstants() {
    return CollectConstantsBuilder.builder()
        .motorCollectRightId(11)
        .motorCollectLeftId(12)
        .collectGearRatio(24.0 / 18.0)
        .collectFeedBackValues(new PidValues(0, 0, 0))
        .build();
  }
}
