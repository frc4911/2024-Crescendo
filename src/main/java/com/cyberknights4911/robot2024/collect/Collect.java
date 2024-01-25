// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.collect;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Collect extends SubsystemBase {
  private final CollectIO collectIO;
  private final CollectIOInputsAutoLogged inputs = new CollectIOInputsAutoLogged();

  public Collect(CollectIO collectIO) {
    super();
    this.collectIO = collectIO;
  }

  public void setVelocity(double velocity) {
    collectIO.setVelocity(velocity);
  }

  @Override
  public void periodic() {
    collectIO.updateInputs(inputs);
    Logger.processInputs("Collect", inputs);
  }

  /** Creates a command that collects at the provided velocity and stops when interrupted. */
  public Command collectFixed(double velocity) {
    return Commands.startEnd(
        () -> {
          setVelocity(velocity);
        },
        () -> {
          setVelocity(0);
        },
        this);
  }

  /**
   * Creates a command collects at the velocity provided by the supplier and stops when interrupted.
   */
  public Command collectVariable(DoubleSupplier velocitySupplier) {
    return Commands.runEnd(
        () -> {
          setVelocity(velocitySupplier.getAsDouble());
        },
        () -> {
          setVelocity(0);
        },
        this);
  }
}
