// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.cyberknights4911.constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;
  private final StatusSignal<Double> roll;
  private final StatusSignal<Double> rollVelocity;

  public GyroIOPigeon2(DriveConstants constants) {
    pigeon = new Pigeon2(constants.pigeonId());

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    roll = pigeon.getRoll();
    rollVelocity = pigeon.getAngularVelocityXWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(100.0);
    yawVelocity.setUpdateFrequency(100.0);

    roll.setUpdateFrequency(100.0);
    rollVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, roll, rollVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(rollVelocity.getValueAsDouble());
  }
}
