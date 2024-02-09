// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.constants;

import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.robot2024.Robot2024Constants;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.function.Supplier;

/**
 * General robot-wide constants. Try not to make this a junk drawer; if constants belong to a
 * subsystem, make a discrete record.
 */
@RecordBuilder
public record Constants(
    String name,
    double loopPeriodSecs,
    boolean tuningMode,
    String logPath,
    Mode mode,
    Supplier<RobotContainer> supplier) {

  // Change the returned value here to switch robots
  public static Constants get() {
    return Robot2024Constants.ROBOT_2024;
  }
}
