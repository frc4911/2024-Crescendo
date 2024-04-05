package com.cyberknights4911.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.cyberknights4911.logging.Mode;
import org.junit.jupiter.api.Test;

public class SparkBurnManagerTest {

  @Test
  public void shouldBurnSim() {
    assertFalse(new SparkBurnManager(Mode.SIM).shouldBurn());
  }

  @Test
  public void shouldBurnReal() {
    assertTrue(new SparkBurnManager(Mode.REAL).shouldBurn());
  }
  
}
