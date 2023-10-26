package com.cyberknights4911.auto;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public final class AutoCommandHandler {
    private final LoggedDashboardChooser<Command> loggedDashboardChooser;
    private double autoStart;
    private boolean autoMessagePrinted;
    private Command currentAutoCommand;

    public AutoCommandHandler() {
        loggedDashboardChooser = new LoggedDashboardChooser<Command>("Auto Routine");
    }

    public void addDefaultOption(String key, Command command) {
        loggedDashboardChooser.addDefaultOption(key, command);
    }

    public void addOption(String key, Command command) {
        loggedDashboardChooser.addOption(key, command);
    }

    public void startCurrentCommand() {
        stopCurrentCommand();
        autoStart = Timer.getFPGATimestamp();
        currentAutoCommand = loggedDashboardChooser.get();
        if (currentAutoCommand != null) {
            currentAutoCommand.schedule();
        }
    }

    public void checkCurrentCommand() {
        // Print auto duration
        if (currentAutoCommand != null) {
            if (!currentAutoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.println(
                        String.format(
                            "*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
                } else {
                    System.out.println(
                        String.format(
                            "*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
                }
                autoMessagePrinted = true;
            }
        }
    }

    public void stopCurrentCommand() {
        if (currentAutoCommand != null) {
            currentAutoCommand.cancel();
        }
    }
}
