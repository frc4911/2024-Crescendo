package com.cyberknights4911.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Factory for creating TalonSRX objects. */
public final class TalonSRXFactory {

    public static TalonSRXFactory createOnRoboRio() {
        return new TalonSRXFactory();
    }

    public WPI_TalonSRX createTalon(int id) {
        return new LazyTalonSRX(id);
    }

    /**
     * Wrapper around the TalonSRX that reduces CAN bus / CPU overhead by skipping duplicate set
     * commands. (By default the Talon flushes the Tx buffer on every set call).
     */
    private static class LazyTalonSRX extends WPI_TalonSRX {
        private double lastSet = Double.NaN;
        private ControlMode lastControlMode = null;
    
        public LazyTalonSRX(int deviceNumber) {
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
