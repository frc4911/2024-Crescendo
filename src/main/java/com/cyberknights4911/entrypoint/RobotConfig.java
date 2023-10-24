package com.cyberknights4911.entrypoint;

import java.util.function.Supplier;
import com.cyberknights4911.logging.Mode;
import com.google.auto.value.AutoValue;

@AutoValue
public abstract class RobotConfig {
  public abstract String name();
  public abstract String logPath();
  public abstract Mode mode();
  public abstract Supplier<RobotContainer> containerSupplier();
}
