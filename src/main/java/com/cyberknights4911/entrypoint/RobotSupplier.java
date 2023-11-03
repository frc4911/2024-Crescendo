package com.cyberknights4911.entrypoint;

import com.cyberknights4911.constants.Constants;
import java.util.function.Supplier;

final class RobotSupplier implements Supplier<Robot> {

  public RobotSupplier() {
  }

  @Override
  public Robot get() {
    return new Robot(Constants.get());
  }
}
