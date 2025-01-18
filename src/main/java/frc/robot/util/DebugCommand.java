package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class DebugCommand {   
    private static ShuffleboardTab tab;

    /**
     * Adds a {@link Command} to the debug tab on {@link ShuffleboardTab} as a button.
     * @param name The name to be displayed for the button
     * @param cmd The {@link Command} to run
     */
    public static void register(String name, Command cmd) {
        if (tab == null) tab = Shuffleboard.getTab("Debug");
        
        tab.add(name, cmd.withName("DEBUG-" + name).ignoringDisable(true)).withWidget("Command");
    }
}
