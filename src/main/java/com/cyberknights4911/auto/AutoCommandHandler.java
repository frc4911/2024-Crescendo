// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import javax.inject.Inject;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoCommandHandler {
  private final LoggedDashboardChooser<Command> loggedDashboardChooser;
  private double autoStart;
  private boolean autoMessagePrinted;
  private Command currentAutoCommand;

  @Inject
  public AutoCommandHandler() {
    loggedDashboardChooser = new LoggedDashboardChooser<Command>("Auto Routine");
  }

  public void addDefaultOption(String key, Command command) {
    loggedDashboardChooser.addDefaultOption(key, command);
  }

  public void addOption(String key, Command command) {
    loggedDashboardChooser.addOption(key, command);
  }

  public void startCurrentCommand() {
    stopCurrentCommand();
    autoStart = Timer.getFPGATimestamp();
    currentAutoCommand = loggedDashboardChooser.get();
    if (currentAutoCommand != null) {
      currentAutoCommand.schedule();
    }
  }

  public void checkCurrentCommand() {
    // Print auto duration
    if (currentAutoCommand != null) {
      if (!currentAutoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }
  }

  public void stopCurrentCommand() {
    if (currentAutoCommand != null) {
      currentAutoCommand.cancel();
    }
  }
}
