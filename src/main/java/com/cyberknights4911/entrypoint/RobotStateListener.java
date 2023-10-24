package com.cyberknights4911.entrypoint;

import org.littletonrobotics.junction.LoggedRobot;

/**
 * Listener interface for receiving callbacks when the robot enters different states.
 * Note that there is no method for onRobotInit because that is when the listener is instantiated.
 */
public interface RobotStateListener {

    void onAutonomousInit(LoggedRobot robot);

    void onAutonomousExit(LoggedRobot robot);

    void onTeleopInit(LoggedRobot robot);

    void onTeleopExit(LoggedRobot robot);
}
