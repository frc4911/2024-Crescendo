// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.ConstantsBuilder;
import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.constants.ControlConstantsBuilder;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.constants.DriveConstantsBuilder;
import com.cyberknights4911.constants.DriveConstantsModuleConstantsBuilder;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.robot2024.climb.ClimbConstants;
import com.cyberknights4911.robot2024.climb.ClimbConstantsBuilder;
import com.cyberknights4911.robot2024.collect.CollectConstants;
import com.cyberknights4911.robot2024.collect.CollectConstantsBuilder;
import com.cyberknights4911.robot2024.shooter.ShooterConstants;
import com.cyberknights4911.robot2024.shooter.ShooterConstantsBuilder;
import com.cyberknights4911.util.FeedForwardValues;
import com.cyberknights4911.util.PidValues;
import edu.wpi.first.math.util.Units;

public final class SimRobot2024Constants {

  private SimRobot2024Constants() {}

  public static final Constants SIM_ROBOT_2024 =
      ConstantsBuilder.builder()
          .name("SimRobot2024")
          .loopPeriodSecs(0.02)
          .tuningMode(true)
          .logPath(null)
          .mode(Mode.SIM)
          .supplier(Robot2024::new)
          .build();

  public static final ControlConstants CONTROL_CONSTANTS =
      ControlConstantsBuilder.builder().driverPort(0).operatorPort(1).stickDeadband(.1).build();

  public static final DriveConstants DRIVE_CONSTANTS =
      DriveConstantsBuilder.builder()
          .maxLinearSpeed(4.5)
          .trackWidthX(Units.inchesToMeters(22.75))
          .trackWidthY(Units.inchesToMeters(22.75))
          .wheelRadius(Units.inchesToMeters(2))
          .turnGearRatio(DriveConstants.TURN_GEAR_RATIO)
          .driveGearRatio(DriveConstants.L3_GEAR_RATIO)
          .pigeonId(0)
          .turnFeedBackValues(new PidValues(0.0, 0.0, 0.0))
          .driveFeedBackValues(new PidValues(0.0, 0.0, 0.0))
          .driveFeedForwardValues(new FeedForwardValues(0.0, 0.0))
          .frontLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontLeft")
                  .driveMotorId(0)
                  .turnMotorId(0)
                  .encoderId(0)
                  .encoderOffset(0.0 - Math.PI)
                  .build())
          .frontRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontRight")
                  .driveMotorId(0)
                  .turnMotorId(0)
                  .encoderId(0)
                  .encoderOffset(0.0 - Math.PI)
                  .build())
          .backLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackLeft")
                  .driveMotorId(0)
                  .turnMotorId(0)
                  .encoderId(0)
                  .encoderOffset(0.0 - Math.PI)
                  .build())
          .backRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackRight")
                  .driveMotorId(0)
                  .turnMotorId(0)
                  .encoderId(0)
                  .encoderOffset(0.0 - Math.PI)
                  .build())
          .build();

  public static ClimbConstants CLIMB_CONSTANTS =
      ClimbConstantsBuilder.builder()
          .leftMotorId(0)
          .rightMotorId(0)
          .gearRatio(1.0)
          .feedBackValues(new PidValues(0, 0, 0))
          .build();

  public static CollectConstants COLLECT_CONSTANTS =
      CollectConstantsBuilder.builder()
          .motorCollectId(11)
          .motorGuideId(12)
          .sensorId(0)
          .solenoidLeftId(0)
          .solenoidRightId(0)
          .collectGearRatio(24.0 / 18.0)
          .collectFeedBackValues(new PidValues(0, 0, 0))
          .guideFeedBackValues(new PidValues(0, 0, 0))
          .build();

  public static ShooterConstants SHOOTER_CONSTANTS =
      ShooterConstantsBuilder.builder()
          .shooterMotorTopId(0)
          .shooterMotorBottomId(0)
          .aimerGearRatio(1.0)
          .feedBackValues(new PidValues(0, 0, 0))
          .feedForwardValues(new FeedForwardValues(0, 0))
          .build();
}
