// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public final class SparkConfig {

  private SparkConfig() {}

  public static void configNotLeader(CANSparkBase spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configLeaderFollower(CANSparkBase spark) {
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }
}
