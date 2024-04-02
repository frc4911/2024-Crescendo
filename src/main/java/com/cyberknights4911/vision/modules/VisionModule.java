// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.vision.modules;

import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.vision.VisionUpdate;
import dagger.Module;
import dagger.Provides;
import java.util.function.Consumer;

@Module
public interface VisionModule {

  @Provides
  public static Consumer<VisionUpdate> providesVisionConsumer(Drive drive) {
    return drive::addVisionMeasurement;
  }
}
