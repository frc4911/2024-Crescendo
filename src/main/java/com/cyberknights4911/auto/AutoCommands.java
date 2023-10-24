package com.cyberknights4911.auto;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoCommands {
    Pair<String, Command> defaultCommand();
    List<Pair<String, Command>> commands();
}
