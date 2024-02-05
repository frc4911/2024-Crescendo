// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.control;

import com.cyberknights4911.constants.ControlConstants;
import com.cyberknights4911.control.ButtonBinding;
import com.cyberknights4911.control.StickBinding;
import com.cyberknights4911.control.Triggers;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

public final class ControllerBinding
    implements StickBinding<StickActions>, ButtonBinding<ButtonActions> {
  private final CommandXboxController driver;
  private final CommandXboxController operator;

  public ControllerBinding(ControlConstants constants) {
    driver = new CommandXboxController(constants.driverPort());
    operator = new CommandXboxController(constants.operatorPort());
  }

  @Override
  public DoubleSupplier supplierFor(StickActions action) {
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
  public Triggers triggersFor(ButtonActions action) {
    switch (action) {
      default:
        return new Triggers(ALWAYS_FALSE);
    }
  }
}
