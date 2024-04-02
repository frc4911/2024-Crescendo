// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.entrypoint;

import com.cyberknights4911.robots.components.DaggerHotwheelsComponent;
import edu.wpi.first.wpilibj.RobotBase;

public final class EntryPoint {
  private EntryPoint() {}

  public static void main(String... args) {
    RobotBase.startRobot(() -> DaggerHotwheelsComponent.create().robot());
  }
}
