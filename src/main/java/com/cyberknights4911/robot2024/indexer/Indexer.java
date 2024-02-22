// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.indexer;

import com.cyberknights4911.logging.LoggedTunableNumber;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public final class Indexer extends SubsystemBase {
  private static final LoggedTunableNumber beamThreshold =
      new LoggedTunableNumber("Indexer/BeamThreshold");
  private static final LoggedTunableNumber speed = new LoggedTunableNumber("Indexer/VelocityRPM");
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("Indexer/Kp");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("Indexer/Kd");

  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private final Command runIndexAtTunableSpeed;
  private final Command runIndexUntilSensor;

  public Indexer(IndexerConstants constants, IndexerIO indexerIO) {
    super();
    this.indexerIO = indexerIO;

    speed.initDefault(constants.speed());
    kp.initDefault(constants.feedBackValues().kP());
    kd.initDefault(constants.feedBackValues().kP());

    indexerIO.configurePID(kp.get(), 0.0, kd.get());

    runIndexAtTunableSpeed = Commands.runOnce(() -> runVelocity(speed.get()), this);

    runIndexUntilSensor =
        runIndexAtTunableSpeed.until(this::isBeamBreakBlocked).andThen(this::stop, this);
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    if (kp.hasChanged(hashCode()) || kd.hasChanged(hashCode())) {
      indexerIO.configurePID(kp.get(), 0.0, kd.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  public boolean isBeamBreakBlocked() {
    return inputs.beamBreakVoltage > beamThreshold.get();
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    indexerIO.setVelocity(velocityRadPerSec);

    Logger.recordOutput("Indexer/SetpointRPM", velocityRpm);
  }

  public void stop() {
    indexerIO.stop();
  }

  /**
   * Returns a command that runs the collector at the tunable speed. This command completes
   * immediately and the indexer will stay running afterward.
   */
  public Command runIndexAtTunableSpeed() {
    return runIndexAtTunableSpeed;
  }

  /**
   * Returns a command that runs the indexer at the tunable speed, but stops when the gamepiece
   * reaches the sensor.
   */
  public Command runIndexUntilSensor() {
    return runIndexUntilSensor;
  }
}
