package com.cyberknights4911.entrypoint;

import org.littletonrobotics.junction.LoggedRobot;

import com.cyberknights4911.auto.AutoCommandHandler;

public interface RobotContainer {
    void onRobotPeriodic(LoggedRobot robot);
    void setupAutos(AutoCommandHandler handler);
}
