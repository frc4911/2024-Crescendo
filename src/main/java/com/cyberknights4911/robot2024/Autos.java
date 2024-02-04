// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.robot2024.climb.Climb;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class Autos {
  private final Climb climb;
  private final Collect collect;
  private final Shooter shooter;
  private final Drive drive;

  public Autos(Climb climb, Collect collect, Shooter shooter, Drive drive) {
    this.climb = climb;
    this.collect = collect;
    this.shooter = shooter;
    this.drive = drive;
  }

  public void addAllAutos(AutoCommandHandler handler) {
    handler.addDefaultOption("Nothing", Commands.none());
    addCharacterization("Climb", climb.getSysId(), handler);
    addCharacterization("Collector", collect.getSysId(), handler);
    addCharacterization("Shooter", shooter.getSysId(), handler);
    addCharacterization("Drive", drive.getSysId(), handler);
  }

  private void addCharacterization(String name, SysIdRoutine routine, AutoCommandHandler handler) {
    handler.addOption(
        name + " SysId (Quasistatic Forward)",
        routine.quasistatic(SysIdRoutine.Direction.kForward));
    handler.addOption(
        name + " SysId (Quasistatic Reverse)",
        routine.quasistatic(SysIdRoutine.Direction.kReverse));
    handler.addOption(
        name + " SysId (Dynamic Forward)", routine.dynamic(SysIdRoutine.Direction.kForward));
    handler.addOption(
        name + " SysId (Dynamic Reverse)", routine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
