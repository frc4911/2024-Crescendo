package com.cyberknights4911.logging;

import com.cyberknights4911.BuildConstants;
import com.cyberknights4911.entrypoint.RobotConfig;
import com.cyberknights4911.logging.Alert.AlertType;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public final class RobotLogger {
  private static final double CAN_ERROR_TIME_THRESHOLD = 0.5; // Seconds to disable alert
  private static final double LOW_BATTERY_VOLTAGE = 10.0;
  private static final double LOW_BATTERY_DISABLED_TIME = 1.5;

  private final Timer canErrorTimer = new Timer();
  private final Timer canErrorTimerInitial = new Timer();
  private final Timer disabledTimer = new Timer();

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);

  private final CommandScheduler scheduler;
  private final RobotConfig config;

  public RobotLogger(RobotConfig config, CommandScheduler scheduler) {
    this.scheduler = scheduler;
    this.config = config;
  }

  public void startLogging(LoggedRobot robot) {
    // Record metadata
    Logger.recordMetadata("Robot", config.name());
    System.out.println("[Init] Starting AdvantageKit");
    Logger.recordMetadata("TuningMode", Boolean.toString(TuningMode.IS_ENABLED));
    Logger.recordMetadata("RuntimeType", LoggedRobot.getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (config.mode()) {
      case REAL:
        String folder = config.logPath();
        if (folder != null) {
          Logger.addDataReceiver(new WPILOGWriter(folder));
        } else {
          logNoFileAlert.set(true);
        }
        Logger.addDataReceiver(new NT4Publisher());
        // TODO set this up if using rev
        // LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    robot.setUseTiming(config.mode() != Mode.REPLAY);
    Logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          // Logger.recordOutput(
          //         "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    scheduler.onCommandInitialize(
      (Command command) -> {
        logCommandFunction.accept(command, true);
      });
    scheduler.onCommandFinish(
      (Command command) -> {
        logCommandFunction.accept(command, false);
      });
    scheduler.onCommandInterrupt(
      (Command command) -> {
        logCommandFunction.accept(command, false);
      });

    // Default to blue alliance in sim
    if (config.mode() == Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    // Start timers
    canErrorTimer.reset();
    canErrorTimer.start();
    canErrorTimerInitial.reset();
    canErrorTimerInitial.start();
    disabledTimer.reset();
    disabledTimer.start();
  }

  public void robotPeriodic() {
    // Check logging fault
    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Update CAN error alert
    var canStatus = RobotController.getCANStatus();
    if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
      canErrorTimer.reset();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD)
            && canErrorTimerInitial.hasElapsed(CAN_ERROR_TIME_THRESHOLD));

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE
        && disabledTimer.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      lowBatteryAlert.set(true);
    }

    // Log list of NT clients
    List<String> clientNames = new ArrayList<>();
    List<String> clientAddresses = new ArrayList<>();
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      clientNames.add(client.remote_id);
      clientAddresses.add(client.remote_ip);
    }
    // Logger.recordOutput("NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
    // Logger.recordOutput(
    //         "NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));
  }
}
