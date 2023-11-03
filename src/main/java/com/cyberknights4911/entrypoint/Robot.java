package com.cyberknights4911.entrypoint;

import com.cyberknights4911.auto.AutoCommandHandler;
import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.logging.RobotLogger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;

public final class Robot extends LoggedRobot {
  private final RobotLogger robotLogger;
  private final AutoCommandHandler autoCommandHandler;
  private final Constants constants;
  private final CommandScheduler scheduler;

  private RobotContainer container;

  public Robot(Constants constants) {
    super();
    this.constants = constants;
    robotLogger = new RobotLogger(constants, CommandScheduler.getInstance());
    autoCommandHandler = new AutoCommandHandler();
    scheduler = CommandScheduler.getInstance();
  }

  @Override
  public void robotInit() {
    robotLogger.startLogging(this);
    container = constants.supplier().get();
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
