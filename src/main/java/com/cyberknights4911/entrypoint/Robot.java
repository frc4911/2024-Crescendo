package com.cyberknights4911.entrypoint;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.logging.RobotLogger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;

public final class Robot extends LoggedRobot {
  private final RobotLogger robotLogger;
  private final AutoCommandHandler autoCommandHandler;
  private final RobotConfig config;
  private final CommandScheduler scheduler;

  private RobotContainer container;

  public Robot(RobotConfig config) {
    super();
    this.config = config;
    robotLogger = new RobotLogger(config, CommandScheduler.getInstance());
    autoCommandHandler = new AutoCommandHandler();
    scheduler = CommandScheduler.getInstance();
  }

  @Override
  public void robotInit() {
    robotLogger.startLogging(this);
    container = config.containerSupplier().get();
    container.setupAutos(autoCommandHandler);
  }

  @Override
  public void robotPeriodic() {
    scheduler.run();
    if (container != null) {
      container.onRobotPeriodic(this);
    }
    autoCommandHandler.checkCurrentCommand();
  }

  @Override
  public void autonomousInit() {
    autoCommandHandler.startCurrentCommand();
  }

  @Override
  public void teleopInit() {
    autoCommandHandler.stopCurrentCommand();
  }
}
