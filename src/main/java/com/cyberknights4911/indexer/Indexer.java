// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.indexer;

import com.cyberknights4911.logging.LoggedTunableNumber;
import com.cyberknights4911.logging.LoggedTunableNumberFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.littletonrobotics.junction.Logger;

@Singleton
public final class Indexer extends SubsystemBase {
  private final LoggedTunableNumber percent;
  private final LoggedTunableNumber kp;
  private final LoggedTunableNumber kd;
  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  @Inject
  public Indexer(
      IndexerConstants constants, IndexerIO indexerIO, LoggedTunableNumberFactory numberFactory) {
    super();
    this.indexerIO = indexerIO;

    percent = numberFactory.getNumber("Indexer/PercentOutput", constants.percentOutput());
    kp = numberFactory.getNumber("Indexer/Kp", constants.feedBackValues().kP());
    kd = numberFactory.getNumber("Indexer/Kd", constants.feedBackValues().kD());

    indexerIO.configurePID(kp.get(), 0.0, kd.get());
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

  public void runVolts(double voltage) {
    indexerIO.setVoltage(voltage);

    Logger.recordOutput("Shooter/IndexerVoltage", voltage);
  }

  public void runOutput(double percent) {
    indexerIO.setOutput(percent);

    Logger.recordOutput("Shooter/IndexerPercent", percent);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRpm) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRpm);
    indexerIO.setVelocity(velocityRadPerSec);

    Logger.recordOutput("Indexer/SetpointRPM", velocityRpm);
  }

  public Command runIndexAtTunableOutput() {
    return Commands.runOnce(() -> runOutput(percent.get()), this);
  }

  public Command stop() {
    return Commands.runOnce(() -> indexerIO.stop(), this);
  }

  public Command runBackwards() {
    return Commands.runOnce(() -> runOutput(-percent.get()), this);
  }
}
