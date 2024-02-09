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
import com.cyberknights4911.robot2024.drive.SparkOdometryThread;
import com.cyberknights4911.wham.drive.PhoenixOdometryThread;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final StatusSignal<Double> yawVelocity;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOPigeon2(DriveConstants driveConstants, PhoenixOdometryThread odometryThread) {
    pigeon = new Pigeon2(driveConstants.pigeonId());
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(driveConstants.odometryFrequency());
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();

    yawTimestampQueue = odometryThread.makeTimestampQueue();
    yawPositionQueue = odometryThread.registerSignal(pigeon, pigeon.getYaw());
  }

  public GyroIOPigeon2(DriveConstants driveConstants, SparkOdometryThread odometryThread) {
    pigeon = new Pigeon2(driveConstants.pigeonId());
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(driveConstants.odometryFrequency());
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();

    yawTimestampQueue = odometryThread.makeTimestampQueue();
    yawPositionQueue = odometryThread.registerSignal(pigeon.getYaw()::getValueAsDouble);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble(value -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
