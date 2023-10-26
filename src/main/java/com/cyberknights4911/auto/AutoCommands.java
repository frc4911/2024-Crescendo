package com.cyberknights4911.auto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

public interface AutoCommands {
    Pair<String, Command> defaultCommand();
    List<Pair<String, Command>> commands();
}
