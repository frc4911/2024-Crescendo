package com.cyberknights4911.ctre;

import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

/** PigeonFactory implementation for creating PigeonIMU objects. */
public final class Pigeon1Factory implements PigeonFactory {
    public static Pigeon1Factory createOnRoboRio() {
        return new Pigeon1Factory();
    }

    private Pigeon1Factory() {}

    @Override
    public BasePigeon create(int id) {
        return new WPI_PigeonIMU(id);
    }
}
