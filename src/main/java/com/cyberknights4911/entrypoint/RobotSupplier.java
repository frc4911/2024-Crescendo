package com.cyberknights4911.entrypoint;

import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

final class RobotSupplier implements Supplier<Robot> {

    private final LoggedDashboardChooser<RobotConfig> chooser;

    public RobotSupplier() {
        chooser = new LoggedDashboardChooser<>("Robot Picker");
        // chooser.addDefaultOption("Wham", () -> new Wham());
    }

    @Override
    public Robot get() {
        return new Robot(chooser.get());
    }
}
