// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.collect.modules;

import com.cyberknights4911.collect.CollectIO;
import com.cyberknights4911.collect.CollectIOReal;
import dagger.Binds;
import dagger.Module;

@Module
public abstract class CollectIORealModule {
  @Binds
  public abstract CollectIO bindsCollectIO(CollectIOReal io);
}
