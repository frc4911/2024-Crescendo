package com.cyberknights4911.entrypoint;

import com.cyberknights4911.auto.AutoCommandHandler;
import org.littletonrobotics.junction.LoggedRobot;

public interface RobotContainer {
    void onRobotPeriodic(LoggedRobot robot);
    void setupAutos(AutoCommandHandler handler);
}
