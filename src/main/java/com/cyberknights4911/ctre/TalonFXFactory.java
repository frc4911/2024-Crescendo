package com.cyberknights4911.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Factory for creating TalonFX objects. */
public final class TalonFXFactory {
  private final String canivoreName;

  public static TalonFXFactory createOnRoboRio() {
    return new TalonFXFactory(null);
  }

  public static TalonFXFactory createOnCanivore(String canivoreName) {
    return new TalonFXFactory(canivoreName);
  }

  private TalonFXFactory(String canivoreName) {
    this.canivoreName = canivoreName;
  }

  public WPI_TalonFX createTalon(int id) {
    if (canivoreName == null || canivoreName.isEmpty()) {
      return new LazyTalonFX(id);
    } else {
      return new LazyTalonFX(id, canivoreName);
    }
  }

  /**
   * Wrapper around the TalonFX that reduces CAN bus / CPU overhead by skipping duplicate set
   * commands. (By default the Talon flushes the Tx buffer on every set call).
   */
  private static class LazyTalonFX extends WPI_TalonFX {
    private double lastSet = Double.NaN;
    private ControlMode lastControlMode = null;

    public LazyTalonFX(int deviceNumber) {
      super(deviceNumber);
    }

    public LazyTalonFX(int deviceNumber, String canbus) {
      super(deviceNumber, canbus);
    }

    @Override
    public void set(ControlMode mode, double value) {
      if (value != lastSet || mode != lastControlMode) {
        lastSet = value;
        lastControlMode = mode;
        super.set(mode, value);
      }
    }
  }
}
