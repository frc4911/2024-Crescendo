// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision.modules;

import com.cyberknights4911.vision.CameraConstants;
import com.cyberknights4911.vision.CameraConstantsBuilder;
import com.cyberknights4911.vision.VisionConstants;
import com.cyberknights4911.vision.VisionConstantsBuilder;
import dagger.Module;
import dagger.Provides;
import dagger.multibindings.IntoSet;

@Module
public interface VisionConstantsSimModule {

  @Provides
  public static VisionConstants providesVisionConstants() {
    return VisionConstantsBuilder.builder().build();
  }

  @Provides
  @IntoSet
  public static CameraConstants providesCameraConstants() {
    return CameraConstantsBuilder.builder().build();
  }
}
