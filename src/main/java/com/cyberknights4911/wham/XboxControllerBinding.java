package com.cyberknights4911.wham;

import java.util.function.DoubleSupplier;
import com.cyberknights4911.control.DriveStickAction;
import com.cyberknights4911.control.StickBinding;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class XboxControllerBinding implements StickBinding {
  private static final int DRIVER_CONTROLLER_PORT = 0;
  private final CommandXboxController driver;

  public XboxControllerBinding() {
    driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  }

  @Override
  public DoubleSupplier supplierFor(DriveStickAction action) {
    switch (action) {
      case FORWARD: return driver::getLeftY;
      case STRAFE: return driver::getLeftX;
      case ROTATE: return driver::getRightX;
      default: return ALWAYS_ZERO;
    }
  }
}
