// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.robot2024.arm.Arm;
import com.cyberknights4911.robot2024.arm.ArmIO;
import com.cyberknights4911.robot2024.climb.Climb;
import com.cyberknights4911.robot2024.climb.ClimbIO;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.collect.CollectIO;
import com.cyberknights4911.robot2024.shooter.Shooter;
import com.cyberknights4911.robot2024.shooter.ShooterIO;
import org.littletonrobotics.junction.LoggedRobot;

/** The main class for the 2024 robot to be named at a future date. */
public final class Robot2024 implements RobotContainer {
  private final Arm arm;
  private final Climb climb;
  private final Collect collect;
  private final Shooter shooter;

  public Robot2024() {
    arm = new Arm(Robot2024Constants.ARM_CONSTANTS, new ArmIO() {});
    climb = new Climb(Robot2024Constants.CLIMB_CONSTANTS, new ClimbIO() {});
    collect = new Collect(Robot2024Constants.COLLECT_CONSTANTS, new CollectIO() {});
    shooter = new Shooter(Robot2024Constants.SHOOTER_CONSTANTS, new ShooterIO() {});
  }

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {}

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    Autos autos = new Autos(arm, collect, shooter);
    autos.addAllAutos(handler);
  }
}
