// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.collect.CollectConstantsBuilder;
import com.cyberknights4911.robot2024.collect.CollectIO;
import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import org.littletonrobotics.junction.LoggedRobot;

/** The main class for the 2024 robot to be named at a future date. */
public final class Robot2024 implements RobotContainer {
  private final Collect collect;

  public Robot2024() {
    collect =
        new Collect(
            CollectConstantsBuilder.builder()
                .motorId(0)
                .sensorId(0)
                .gearRatio(1.0)
                .feedBackValues(new PidValues(0, 0, 0))
                .feedForwardValues(new FeedForwardValues(0, 0))
                .build(),
            new CollectIO() {});
  }

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {}

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    Autos autos = new Autos(collect);
    autos.addAllAutos(handler);
  }
}
