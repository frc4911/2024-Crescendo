// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.drive.Drive;
import com.cyberknights4911.drive.GyroIO;
import com.cyberknights4911.drive.ModuleIO;
import com.cyberknights4911.drive.ModuleIOSim;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.robot2024.arm.Arm;
import com.cyberknights4911.robot2024.arm.ArmIO;
import com.cyberknights4911.robot2024.arm.ArmIOSim;
import com.cyberknights4911.robot2024.climb.Climb;
import com.cyberknights4911.robot2024.climb.ClimbIO;
import com.cyberknights4911.robot2024.climb.ClimbIOSim;
import com.cyberknights4911.robot2024.collect.Collect;
import com.cyberknights4911.robot2024.collect.CollectIO;
import com.cyberknights4911.robot2024.collect.CollectIOSim;
import com.cyberknights4911.robot2024.shooter.Shooter;
import com.cyberknights4911.robot2024.shooter.ShooterIO;
import com.cyberknights4911.robot2024.shooter.ShooterIOSim;
import org.littletonrobotics.junction.LoggedRobot;

/** The main class for the 2024 robot to be named at a future date. */
public final class Robot2024 implements RobotContainer {
  private final Arm arm;
  private final Climb climb;
  private final Collect collect;
  private final Shooter shooter;
  private final Constants constants;
  private final Drive drive;

  public Robot2024() {
    constants = Constants.get();
    arm = createArm();
    climb = createClimb();
    collect = createCollect();
    shooter = createShooter();
    drive = createDrive();
  }

  @Override
  public void onRobotPeriodic(LoggedRobot robot) {}

  @Override
  public void setupAutos(AutoCommandHandler handler) {
    Autos autos = new Autos(arm, climb, collect, shooter, drive);
    autos.addAllAutos(handler);
  }

  private Arm createArm() {
    switch (constants.mode()) {
      case SIM:
        return new Arm(
            SimRobot2024Constants.ARM_CONSTANTS, new ArmIOSim(SimRobot2024Constants.ARM_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Arm(Robot2024Constants.ARM_CONSTANTS, new ArmIO() {});
    }
  }

  private Climb createClimb() {
    switch (constants.mode()) {
      case SIM:
        return new Climb(
            SimRobot2024Constants.CLIMB_CONSTANTS,
            new ClimbIOSim(SimRobot2024Constants.CLIMB_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Climb(Robot2024Constants.CLIMB_CONSTANTS, new ClimbIO() {});
    }
  }

  private Collect createCollect() {
    switch (constants.mode()) {
      case SIM:
        return new Collect(
            SimRobot2024Constants.COLLECT_CONSTANTS,
            new CollectIOSim(SimRobot2024Constants.COLLECT_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Collect(Robot2024Constants.COLLECT_CONSTANTS, new CollectIO() {});
    }
  }

  private Shooter createShooter() {
    switch (constants.mode()) {
      case SIM:
        return new Shooter(
            SimRobot2024Constants.SHOOTER_CONSTANTS,
            new ShooterIOSim(SimRobot2024Constants.SHOOTER_CONSTANTS));
      case REAL:
      case REPLAY:
      default:
        return new Shooter(Robot2024Constants.SHOOTER_CONSTANTS, new ShooterIO() {});
    }
  }

  private Drive createDrive() {
    switch (constants.mode()) {
      case SIM:
        return new Drive(
            constants,
            Robot2024Constants.DRIVE_CONSTANTS,
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
      case REAL:
      case REPLAY:
      default:
        return new Drive(
            constants,
            Robot2024Constants.DRIVE_CONSTANTS,
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }
  }
}
