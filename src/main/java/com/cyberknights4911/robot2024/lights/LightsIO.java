// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.robot2024.lights;

public interface LightsIO {
  public default void setHueVoltage(double voltage) {}

  public default void setPatternVoltage(double voltage) {}
}
