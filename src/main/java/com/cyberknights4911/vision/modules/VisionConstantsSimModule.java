// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision.modules;

import com.cyberknights4911.util.Field;
import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.VisionConstants;
import com.cyberknights4911.vision.VisionConstantsBuilder;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.ElementsIntoSet;
import java.util.Collections;
import java.util.Set;

@Module
public interface VisionConstantsSimModule {

  @Provides
  public static VisionConstants providesVisionConstants(Field field) {
    return VisionConstantsBuilder.builder()
        .layout(field.getFieldLayout())
        .maxAmbiguity(0.03)
        .maxValidDistanceMeters(3.0)
        .build();
  }

  @Provides
  @ElementsIntoSet
  public static Set<CameraConstants> providesCameraConstants() {
    return Collections.emptySet();
  }
}
