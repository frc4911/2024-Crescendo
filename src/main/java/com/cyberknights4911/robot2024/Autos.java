// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.commands.FeedForwardCharacterization;
import com.cyberknights4911.robot2024.arm.Arm;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.shooter.Shooter;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  private final Arm arm;
  private final Collect collect;
  private final Shooter shooter;

  public Autos(Arm arm, Collect collect, Shooter shooter) {
    this.arm = arm;
    this.collect = collect;
    this.shooter = shooter;
  }

  public void addAllAutos(AutoCommandHandler handler) {
    NamedCommands.registerCommand("CollectMedium", collect.collectMedium());
    NamedCommands.registerCommand("CollectSlow", collect.collectSlow());

    handler.addDefaultOption("Nothing", Commands.none());
    handler.addOption(
        "Collector FF Characterization",
        new FeedForwardCharacterization(
            collect, collect::runCharacterizationVolts, collect::getCharacterizationVelocity));
    handler.addOption(
        "Shooter FF Characterization",
        new FeedForwardCharacterization(
            shooter, shooter::runCharacterizationVolts, shooter::getCharacterizationVelocity));
  }
}
