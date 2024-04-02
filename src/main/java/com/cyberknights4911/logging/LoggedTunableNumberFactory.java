// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.logging;

import java.util.HashMap;
import java.util.Map;
import javax.inject.Inject;
import javax.inject.Named;
import javax.inject.Singleton;

@Singleton
public final class LoggedTunableNumberFactory {
  private final boolean tuningMode;
  private final Map<String, LoggedTunableNumber> numbers;

  @Inject
  public LoggedTunableNumberFactory(@Named("TuningMode") boolean tuningMode) {
    this.tuningMode = tuningMode;
    numbers = new HashMap<>();
  }

  public LoggedTunableNumber getNumber(String key) {
    return numbers.computeIfAbsent(
        key, dashboardKey -> new LoggedTunableNumber(dashboardKey, tuningMode));
  }

  public LoggedTunableNumber getNumber(String key, double defaultValue) {
    return numbers.computeIfAbsent(
        key, dashboardKey -> new LoggedTunableNumber(dashboardKey, defaultValue, tuningMode));
  }
}
