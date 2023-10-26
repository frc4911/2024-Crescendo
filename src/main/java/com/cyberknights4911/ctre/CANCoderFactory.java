package com.cyberknights4911.ctre;

import com.ctre.phoenix.sensors.WPI_CANCoder;

/** Factory for creating  CANCoder objects. */
public final class CANCoderFactory {
    private final String canivoreName;

    public static CANCoderFactory createOnRoboRio() {
        return new CANCoderFactory(null);
    }

    public static CANCoderFactory createOnCanivore(String canivoreName) {
        return new CANCoderFactory(canivoreName);
    }
    
    private CANCoderFactory(String canivoreName) {
        this.canivoreName = canivoreName;
    }

    public WPI_CANCoder create(int id) {
        if (canivoreName == null || canivoreName.isEmpty()) {
            return new WPI_CANCoder(id);
        } else {
            return new WPI_CANCoder(id, canivoreName);
        }
    }
}
