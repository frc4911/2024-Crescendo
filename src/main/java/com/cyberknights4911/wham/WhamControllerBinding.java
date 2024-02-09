// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.wham;

import com.cyberknights4911.control.ButtonBinding;
import com.cyberknights4911.control.StickBinding;
import com.cyberknights4911.control.Triggers;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public final class WhamControllerBinding
    implements StickBinding<WhamSticks>, ButtonBinding<WhamButtons> {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController driver;

  public WhamControllerBinding() {
    driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  }

  @Override
  public DoubleSupplier supplierFor(WhamSticks action) {
    switch (action) {
      case FORWARD:
        return () -> -driver.getLeftY();
      case STRAFE:
        return () -> -driver.getLeftX();
      case ROTATE:
        return () -> -driver.getRightX();
      default:
        return ALWAYS_ZERO;
    }
  }

  @Override
  public Triggers triggersFor(WhamButtons action) {
    switch (action) {
      case Brake:
        return new Triggers(driver.x());
      case DriveForwardSlow:
        return new Triggers(driver.povUp());
      case DriveBackwardSlow:
        return new Triggers(driver.povDown());
      case ZeroGyro:
        return new Triggers(driver.y());
      case SimulateCollect:
        return new Triggers(driver.rightBumper());
      case SimulateScore:
        return new Triggers(driver.rightTrigger());
      default:
        return new Triggers(ALWAYS_FALSE);
    }
  }
}
