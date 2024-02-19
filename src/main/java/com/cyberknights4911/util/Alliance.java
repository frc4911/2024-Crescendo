// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.util;

import edu.wpi.first.wpilibj.DriverStation;

public final class Alliance {
  private Alliance() {}

  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)
        == DriverStation.Alliance.Red;
  }
}
