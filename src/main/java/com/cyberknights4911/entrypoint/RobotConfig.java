package com.cyberknights4911.entrypoint;

import com.cyberknights4911.logging.Mode;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.function.Supplier;

@RecordBuilder
public record RobotConfig(
  String name,
  String logPath,
  Mode mode,
  Supplier<RobotContainer> supplier) {}
  