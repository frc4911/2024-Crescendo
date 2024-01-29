// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.arm;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public final class AimContinuousCommand extends Command {
  private final Arm arm;
  private final DoubleSupplier distanceSupplier;

  private double distanceMeters;

  AimContinuousCommand(Arm arm, DoubleSupplier distanceSupplier) {
    addRequirements(arm);
    this.arm = arm;
    this.distanceSupplier = distanceSupplier;
  }

  @Override
  public void execute() {
    double distanceMeters = distanceSupplier.getAsDouble();
    // replace this with a call to move the arm to the optimal angle for the latest distance.
  }

  public boolean isLockedOn() {
    // replace this with an expression that returns true when this arm is in a good position to fire
    return false;
  }
}
