// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.climb.modules;

import com.cyberknights4911.climb.ClimbIO;
import com.cyberknights4911.climb.ClimbIOReal;
import dagger.Binds;
import dagger.Module;

@Module
public abstract class ClimbIORealModule {

  @Binds
  public abstract ClimbIO bindsClimbIO(ClimbIOReal real);
}
