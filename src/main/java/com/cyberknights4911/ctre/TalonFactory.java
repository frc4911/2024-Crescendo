package com.cyberknights4911.ctre;

import com.ctre.phoenix6.hardware.TalonFX;

/** Factory for creating TalonFX objects. */
public final class TalonFactory {
  private final String canivoreName;

  public static TalonFactory createOnRoboRio() {
    return new TalonFactory(null);
  }

  public static TalonFactory createOnCanivore(String canivoreName) {
    return new TalonFactory(canivoreName);
  }

  private TalonFactory(String canivoreName) {
    this.canivoreName = canivoreName;
  }

  public TalonFX create(int id) {
    if (canivoreName == null || canivoreName.isEmpty()) {
      return new TalonFX(id);
    } else {
      return new TalonFX(id, canivoreName);
    }
  }
}
