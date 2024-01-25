// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.commands.FeedForwardCharacterization;
import com.cyberknights4911.robot2024.collect.Collect;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  private final Collect collect;

  public Autos(Collect collect) {
    this.collect = collect;
  }

  public void addAllAutos(AutoCommandHandler handler) {
    NamedCommands.registerCommand("CollectMedium", collect.collectMedium());
    NamedCommands.registerCommand("CollectSlow", collect.collectSlow());

    handler.addDefaultOption("Nothing", Commands.none());
    handler.addOption(
        "Collector FF Characterization",
        new FeedForwardCharacterization(
            collect, collect::runCharacterizationVolts, collect::getCharacterizationVelocity));
  }
}
