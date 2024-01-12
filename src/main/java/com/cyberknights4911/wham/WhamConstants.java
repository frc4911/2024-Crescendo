// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.constants.ConstantsBuilder;
import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.constants.ControlConstantsBuilder;
import com.cyberknights4911.constants.DriveConstants;
import com.cyberknights4911.constants.DriveConstantsBuilder;
import com.cyberknights4911.constants.DriveConstantsModuleConstantsBuilder;
import com.cyberknights4911.logging.Mode;
import edu.wpi.first.math.util.Units;

public final class WhamConstants {
  private WhamConstants() {}

  public static final Constants WHAM =
      ConstantsBuilder.builder()
          .name("Wham")
          .loopPeriodSecs(0.02)
          .tuningMode(true)
          .logPath("/media/sda1/")
          .mode(Mode.REAL)
          .supplier(Wham::new)
          .build();

  public static final ControlConstants CONTROL_CONSTANTS =
      ControlConstantsBuilder.builder()
          .driverControllerPort(0)
          .operatorControllerPort(1)
          .stickDeadband(.1)
          .build();

  public static final DriveConstants DRIVE_CONSTANTS =
      DriveConstantsBuilder.builder()
          .maxLinearSpeed(4.5)
          .trackWidthX(Units.inchesToMeters(22.75))
          .trackWidthY(Units.inchesToMeters(22.75))
          .wheelRadius(Units.inchesToMeters(2))
          .treadWear(0)
          .turnGearRatio(DriveConstants.TURN_GEAR_RATIO)
          .driveGearRatio(DriveConstants.L1_GEAR_RATIO)
          .frontLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontLeft")
                  .driveMotorId(2)
                  .turnMotorId(6)
                  .encoderId(1)
                  .encoderOffset(0.360 - Math.PI)
                  .build())
          .frontRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("FrontRight")
                  .driveMotorId(1)
                  .turnMotorId(5)
                  .encoderId(0)
                  .encoderOffset(2.769 - Math.PI)
                  .build())
          .backLeft(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackLeft")
                  .driveMotorId(3)
                  .turnMotorId(7)
                  .encoderId(2)
                  .encoderOffset(-2.946 - Math.PI)
                  .build())
          .backRight(
              DriveConstantsModuleConstantsBuilder.builder()
                  .name("BackRight")
                  .driveMotorId(4)
                  .turnMotorId(8)
                  .encoderId(3)
                  .encoderOffset(1.804 - Math.PI)
                  .build())
          .build();
}
