// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.climb;

import com.cyberknights4911.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;

final class ClimbCommand extends Command {
  private final Drive drive;
  private final Climb climb;

  private boolean isFinished = false;

  ClimbCommand(Drive drive, Climb climb) {
    this.drive = drive;
    this.climb = climb;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    if (climb.isClimbComplete()) {
      isFinished = true;
      return;
    }
    double rollDegrees = drive.getRoll().getDegrees();
    if (Math.abs(rollDegrees) < 0.1) {
      // Robot is level. React accordingly
    } else if (rollDegrees < 0) {
      // Robot is tilting left. React accordingly
    } else {
      // Robot is tilting right. React accordingly
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
