package com.cyberknights4911.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/** Factory for creating VictorSPX objects. */
public final class VictorSPXFactory {

  public static VictorSPXFactory createOnRoboRio() {
    return new VictorSPXFactory();
  }

  public WPI_VictorSPX createTalon(int id) {
    return new LazyVictorSPX(id);
  }

  /**
   * Wrapper around the VictorSPX that reduces CAN bus / CPU overhead by skipping duplicate set
   * commands. (By default the Talon flushes the Tx buffer on every set call).
   */
  private static class LazyVictorSPX extends WPI_VictorSPX {
    private double lastSet = Double.NaN;
    private ControlMode lastControlMode = null;

    public LazyVictorSPX(int deviceNumber) {
      super(deviceNumber);
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
