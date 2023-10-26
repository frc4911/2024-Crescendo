package com.cyberknights4911.ctre;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

/** PigeonFactory implementation for creating Pigeon2 objects. */
public final class Pigeon2Factory implements PigeonFactory {
    private final String canivoreName;

    public static Pigeon2Factory createOnRoboRio() {
        return new Pigeon2Factory(null);
    }

    public static Pigeon2Factory createOnCanivore(String canivoreName) {
        return new Pigeon2Factory(canivoreName);
    }

    private Pigeon2Factory(String canivoreName) {
        this.canivoreName = canivoreName;
    }

    @Override
    public BasePigeon create(int id) {
        if (canivoreName == null || canivoreName.isEmpty()) {
            return new WPI_Pigeon2(id);
        } else {
            return new WPI_Pigeon2(id, canivoreName);
        }
    }
}
