// Copyright (c) 2024 FRC 4911
// https://github.com/frc4911
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.cyberknights4911.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import com.cyberknights4911.logging.Mode;
import com.revrobotics.CANSparkBase;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class SparkBurnManagerTest {

  @Mock private CANSparkBase spark;
  @Mock private Runnable run;

  @Test
  public void shouldBurnSim() {
    assertFalse(new SparkBurnManager(Mode.SIM).shouldBurn());
  }

  @Test
  public void shouldBurnReal() {
    assertTrue(new SparkBurnManager(Mode.REAL).shouldBurn());
  }

  @Test
  public void maybeBurnConfigSim() {
    SparkBurnManager manager = new SparkBurnManager(Mode.SIM);

    manager.maybeBurnConfig(run, spark);

    verify(spark, never()).restoreFactoryDefaults();
    verify(spark, times(2)).setCANTimeout(anyInt());
    verify(spark, never()).burnFlash();
    verify(run, times(4)).run();
  }

  @Test
  public void maybeBurnConfigReal() {
    SparkBurnManager manager = new SparkBurnManager(Mode.REAL);

    manager.maybeBurnConfig(run, spark);

    verify(spark, times(1)).restoreFactoryDefaults();
    verify(spark, times(2)).setCANTimeout(anyInt());
    verify(spark, times(1)).burnFlash();
    verify(run, times(4)).run();
  }
}
