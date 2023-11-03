package com.cyberknights4911.ctre;

import com.ctre.phoenix6.hardware.Pigeon2;

/** Factory for creating Pigeon2 objects. */
public final class PigeonFactory {
  private final String canivoreName;

  public static PigeonFactory createOnRoboRio() {
    return new PigeonFactory(null);
  }

  public static PigeonFactory createOnCanivore(String canivoreName) {
    return new PigeonFactory(canivoreName);
  }

  private PigeonFactory(String canivoreName) {
    this.canivoreName = canivoreName;
  }

  public Pigeon2 create(int id) {
    if (canivoreName == null || canivoreName.isEmpty()) {
      return new Pigeon2(id);
    } else {
      return new Pigeon2(id, canivoreName);
    }
  }
}
