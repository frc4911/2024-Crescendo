// Copyright (c) 2023 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.util;

import com.cyberknights4911.BuildConstants;
import com.cyberknights4911.constants.Constants;
import com.cyberknights4911.logging.Mode;
import com.revrobotics.CANSparkBase;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

/** Determines whether to burn Spark configs to flash. */
public final class SparkBurnManager {
  public static final int CAN_TIMEOUT = 500;
  // How many times to set the config on init
  public static final int CONFIG_ATTEMPT_COUNT = 4;
  private static final String BUILD_DATE_FILE = "/home/lvuser/build-date.txt";

  private final boolean shouldBurn;

  public SparkBurnManager(Constants constants) {
    if (constants.mode() == Mode.SIM) {
      shouldBurn = false;
      return;
    }

    File file = new File(BUILD_DATE_FILE);
    if (!file.exists()) {

      // No build date file, burn flash
      shouldBurn = true;
    } else {

      // Read previous build date
      String previousBuildDate = "";
      try {
        previousBuildDate =
            new String(Files.readAllBytes(Paths.get(BUILD_DATE_FILE)), StandardCharsets.UTF_8);
      } catch (IOException e) {
        e.printStackTrace();
      }

      shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
    }

    try {
      FileWriter fileWriter = new FileWriter(BUILD_DATE_FILE);
      fileWriter.write(BuildConstants.BUILD_DATE);
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (shouldBurn) {
      System.out.println("[SparkBurnManager] Build date changed, burning Spark flash");
    } else {
      System.out.println("[SparkBurnManager] Build date unchanged, will not burn Spark flash");
    }
  }

  public boolean shouldBurn() {
    return shouldBurn;
  }

  public void maybeBurnConfig(Runnable runnable, CANSparkBase... devices) {
    if (shouldBurn) {
      for (CANSparkBase device : devices) {
        device.restoreFactoryDefaults();
      }
    }

    // Make CAN messsages blocking for configuration
    for (CANSparkBase device : devices) {
      device.setCANTimeout(CAN_TIMEOUT);
    }

    // Attempt performing the config block a fixed number of times
    for (int i = 0; i < CONFIG_ATTEMPT_COUNT; i++) {
      runnable.run();
    }

    // Make CAN messsages async for normal operation
    for (CANSparkBase device : devices) {
      device.setCANTimeout(0);
    }

    if (shouldBurn) {
      for (CANSparkBase device : devices) {
        device.burnFlash();
      }
    }
  }
}
