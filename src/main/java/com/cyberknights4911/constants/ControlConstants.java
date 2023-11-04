package com.cyberknights4911.constants;

import java.util.HashMap;
import java.util.Map;
import io.soabase.recordbuilder.core.RecordBuilder;

@RecordBuilder
public record  ControlConstants(
  int driverControllerPort,
  double stickDeadband
) {

  public static ControlConstants get() {
    ControlConstants constants = ROBOTS.get(Constants.get().name());
    if (constants == null) {
      throw new IllegalStateException(
        "No ControlConstants defined for robot named: " + Constants.get().name());
    }
    return constants;
  }

  private static Map<String, ControlConstants> ROBOTS = new HashMap<>();
  static {
    ROBOTS.put(Constants.WHAM.name(), getWhamConstants());
    // TODO: put others here too
  }

  private static ControlConstants getWhamConstants() {
    return ControlConstantsBuilder.builder()
      .driverControllerPort(0)
      .stickDeadband(.01)
      .build();
  }
}
