package com.cyberknights4911.ctre;

import com.ctre.phoenix6.hardware.CANcoder;

/** Factory for creating CANcoder objects. */
public final class CANcoderFactory {
  private final String canivoreName;

  public static CANcoderFactory createOnRoboRio() {
    return new CANcoderFactory(null);
  }

  public static CANcoderFactory createOnCanivore(String canivoreName) {
    return new CANcoderFactory(canivoreName);
  }

  private CANcoderFactory(String canivoreName) {
    this.canivoreName = canivoreName;
  }

  public CANcoder create(int id) {
    if (canivoreName == null || canivoreName.isEmpty()) {
      return new CANcoder(id);
    } else {
      return new CANcoder(id, canivoreName);
    }
  }
}
