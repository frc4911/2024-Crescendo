// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public final class AimContinuousCommand extends Command {
  private final Shooter shooter;
  private final DoubleSupplier distanceSupplier;

  private double distanceMeters;

  AimContinuousCommand(Shooter shooter, DoubleSupplier distanceSupplier) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.distanceSupplier = distanceSupplier;
  }

  @Override
  public void execute() {
    double distanceMeters = distanceSupplier.getAsDouble();
    // replace this with a call to move the shooter to the optimal angle for the latest distance.
  }

  /**
   * Use this method to see if the shooter is at an optimal angle for firing. Note that optimal is
   * momentary; this method must be continuously checked if the robot is moving.
   */
  public boolean isLockedOn() {
    // replace this with an expression that returns true when this shooter is in a good position to
    // fire
    return false;
  }
}
