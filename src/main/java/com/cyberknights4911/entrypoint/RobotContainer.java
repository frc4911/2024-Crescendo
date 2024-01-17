// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.entrypoint;

import com.cyberknights4911.auto.AutoCommandHandler;
import org.littletonrobotics.junction.LoggedRobot;

public interface RobotContainer {
  void onRobotPeriodic(LoggedRobot robot);

  void setupAutos(AutoCommandHandler handler);
}
