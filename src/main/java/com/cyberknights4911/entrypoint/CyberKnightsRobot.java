// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.entrypoint;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.logging.Alert;
import com.cyberknights4911.logging.Alert.AlertType;
import com.cyberknights4911.logging.RobotLogger;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import javax.inject.Inject;
import javax.inject.Named;
import javax.inject.Provider;
import org.littletonrobotics.junction.LoggedRobot;

public final class CyberKnightsRobot extends LoggedRobot {
  private final RobotLogger robotLogger;
  private final AutoCommandHandler autoCommandHandler;
  private final CommandScheduler scheduler;
  private final boolean tuningMode;
  private final Provider<RobotContainer> containerProvider;

  private RobotContainer container;

  @Inject
  public CyberKnightsRobot(
      RobotLogger robotLogger,
      AutoCommandHandler autoCommandHandler,
      CommandScheduler scheduler,
      @Named("TuningMode") boolean tuningMode,
      Provider<RobotContainer> containerProvider) {
    super();
    this.robotLogger = robotLogger;
    this.autoCommandHandler = autoCommandHandler;
    this.scheduler = scheduler;
    this.tuningMode = tuningMode;
    this.containerProvider = containerProvider;
  }

  @Override
  public void robotInit() {
    robotLogger.startLogging(this);
    container = containerProvider.get();
    container.setupAutos(autoCommandHandler);
    checkStartupAlerts();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    scheduler.run();
    if (container != null) {
      container.onRobotPeriodic(this);
    }
    robotLogger.robotPeriodic();
    autoCommandHandler.checkCurrentCommand();
    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void autonomousInit() {
    autoCommandHandler.startCurrentCommand();
  }

  @Override
  public void teleopInit() {
    autoCommandHandler.stopCurrentCommand();
  }

  private void checkStartupAlerts() {
    if (tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", AlertType.INFO).set(true);
    }
  }
}
