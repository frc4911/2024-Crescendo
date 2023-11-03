package com.cyberknights4911.constants;

import java.util.function.Supplier;
import com.cyberknights4911.entrypoint.RobotContainer;
import com.cyberknights4911.logging.Mode;
import com.cyberknights4911.simbot.SimBot;
import com.cyberknights4911.wham.Wham;

import io.soabase.recordbuilder.core.RecordBuilder;

/**
 * General robot-wide constants. Try not to make this a junk drawer; if constants belong to a
 * subsystem, make a discrete record.
 */
@RecordBuilder
public record Constants(
  String name,
  double loopPeriodSecs,
  boolean tuningMode,
  String logPath,
  Mode mode,
  Supplier<RobotContainer> supplier
) {

  // Change the returned value here to switch robots
  public static Constants get() {
    return WHAM;
  }

  final static Constants SIM_BOT = ConstantsBuilder.builder()
    .name("SimBot")
    .loopPeriodSecs(0.02)
    .tuningMode(false)
    .logPath(null)
    .mode(Mode.SIM)
    .supplier(() -> new SimBot())
    .build();

  final static Constants WHAM = ConstantsBuilder.builder()
    .name("Wham")
    .loopPeriodSecs(0.02)
    .tuningMode(false)
    .logPath(null)
    .mode(Mode.REAL)
    .supplier(() -> new Wham())
    .build();

  // TODO: add other robots here
}
